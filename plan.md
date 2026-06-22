## 1. Design Decisions and Rationale

### 1.1 Core Design Choices

| Decision | Choice | Reason |
| --- | --- | --- |
| DMA staging | Single 27-byte buffer | `spi_busy` guarantees only one in-flight transaction at a time; ping-pong is redundant |
| Transaction struct | Configured once; never modified at runtime | `rx_buffer` and `user` are fixed — ISR just re-queues the same struct |
| Parse location | Inside `post_cb`, before clearing `spi_busy` | DMA buffer is stable during parse; clears before next ISR; keeps handler task logic simple |
| Ring buffer unit | One **chunk** per slot (parsed samples) | Enables zero-copy `on_chunk` delivery; natural capacity semantics |
| Ring buffer type | Custom SPSC; capacity-1 usable slots | Avoids lock; standard sentinel-slot full/empty disambiguation |
| Overflow policy | Drop newest; count in `overflow_count` | Preserves SPSC invariant (ISR never touches `tail`); user detects loss via counter |
| Task notification | `xTaskNotifyFromISR` / `eSetValueWithOverwrite` | Lightweight; coalesces redundant wakeups automatically |
| SPI ret_queue drain | Handler task; loop with timeout 0 | Keeps ESP-IDF SPI driver's internal queue from filling; all data work already done in `post_cb` |
| SPI `queue_size` | Set to `chunk_samples` in `start_continuous` | Prevents ret_queue overflow between handler task wake-ups |
| Device re-add | `spi_bus_add_device()` / `spi_bus_remove_device()` in `start_continuous()` / `stop_continuous()` | `queue_size` must match `chunk_samples`, which is not known until `start_continuous()`; `init()` uses `queue_size = 1` for polling |

## 2. Data Structures

### 2.1 `ads1299_chunkring_t` — Ring Buffer of Parsed Chunks

```c
/*
 * Circular buffer of fully parsed ADS1299 chunks.
 *
 * Layout of buf[]:
 *
 *   slot 0                    slot 1
 *   [s0|s1|...|sN-1]          [s0|s1|...|sN-1]    ...
 *   ^                          ^
 *   buf[0]                     buf[chunk_samples]
 *
 * A slot is addressed as: buf[(index & mask) * chunk_samples + sample_i]
 *
 * capacity slots are physically allocated.
 * capacity-1 slots are usable (one sentinel slot disambiguates full from empty).
 *
 * SPSC ownership:
 *   head — written exclusively by ISR (post_cb)
 *   tail — written exclusively by handler task
 */
typedef struct {
    ads1299_sample_t *buf;           /* flat array: capacity * chunk_samples entries  */
    uint32_t          capacity;      /* total slot count; must be power-of-2, >= 2   */
    uint32_t          chunk_samples; /* samples per slot                              */
    uint32_t          mask;          /* capacity - 1; used for slot index modulo      */
    volatile uint32_t head;          /* next write slot index; ISR-owned             */
    volatile uint32_t tail;          /* next read slot index; task-owned             */
} ads1299_chunkring_t;
```

**Slot state invariants:**
- `head == tail` → ring is empty
- `head - tail == capacity - 1` → ring is full (sentinel)
- `head - tail` → number of committed (readable) chunks
- `(capacity - 1) - (head - tail)` → free slots available to commit into

`head` and `tail` are declared `volatile uint32_t` and treated as unsigned 32-bit counters that wrap naturally. The difference `head - tail` always yields the correct available count under unsigned arithmetic regardless of wraparound.

---

### 2.2 `ads1299_dma_ctx_t` — Acquisition Context (updated)

```c
struct ads1299_dma_ctx {

    /* ── DMA staging ─────────────────────────────────────────────── */
    uint8_t           *dma_buf;           /* 27-byte receive buffer; MALLOC_CAP_DMA  */
    spi_transaction_t  trans;             /* reused for every sample; configured once */
    volatile bool      spi_busy;          /* true while DMA in-flight                */
    int64_t            current_timestamp; /* DRDY edge time; written ISR, read post_cb */

    /* ── Ring buffer ──────────────────────────────────────────────── */
    ads1299_chunkring_t ring;
    uint32_t            sample_count;     /* samples written into ring's current slot */

    /* ── Handler task ─────────────────────────────────────────────── */
    TaskHandle_t        handler_task;
    SemaphoreHandle_t   done_sem;         /* task gives this before vTaskDelete       */
    volatile bool       stop_requested;

    /* ── Callbacks ────────────────────────────────────────────────── */
    ads1299_chunk_cb_t  on_chunk;
    ads1299_error_cb_t  on_error;
    void               *cb_ctx;

    /* ── Statistics ───────────────────────────────────────────────── */
    volatile uint32_t   dropped_count;    /* DRDY pulses dropped; spi_busy was set   */
    volatile uint32_t   overflow_count;   /* chunks dropped; ring was full            */
};
```

**`trans` field setup (once, inside `start_continuous`):**
```c
memset(&ctx->trans, 0, sizeof(spi_transaction_t));
ctx->trans.length    = ADS1299_FRAME_SIZE * 8;   /* total bits */
ctx->trans.rxlength  = ADS1299_FRAME_SIZE * 8;
ctx->trans.tx_buffer = NULL;
ctx->trans.rx_buffer = ctx->dma_buf;             /* fixed; never changes */
ctx->trans.user      = ctx;                       /* fixed; never changes */
```

The ISR passes `&ctx->trans` to `spi_device_queue_trans` every DRDY cycle without modifying any field. `rx_buffer` always points to `ctx->dma_buf`. `post_cb` always receives the same `ctx` from `trans->user`.

---

### 2.3 `ads1299_continuous_config_t` — New Field

```c
typedef struct {
    ads1299_chunk_cb_t on_chunk;
    ads1299_error_cb_t on_error;
    void              *ctx;
    uint32_t           chunk_duration_ms;

    /**
     * Ring buffer depth in number of chunk slots.
     * 0 = ADS1299_RING_BUF_SLOTS (default).
     *
     * Must be a power of 2. Rounded up internally if not.
     *
     * Total memory allocated:
     *   ring_buffer_chunks * chunk_samples * sizeof(ads1299_sample_t)
     *
     * At 250 SPS, 100 ms chunks, 8 slots:
     *   8 * 25 * 44 = 8,800 bytes
     *
     * At 16 kSPS, 10 ms chunks, 8 slots:
     *   8 * 160 * 44 = 56,320 bytes
     *
     * At 16 kSPS, 100 ms chunks, 8 slots:
     *   8 * 1600 * 44 = 563,200 bytes  ← EXCEEDS ESP32 SRAM; reduce chunk_duration_ms
     */
    uint32_t    ring_buffer_chunks;

    UBaseType_t task_priority;
    BaseType_t  task_core;
} ads1299_continuous_config_t;
```

---

### 2.4 Memory Sizing at Common Configurations

`sizeof(ads1299_sample_t)` = 44 bytes (3 status + 1 pad + 32 channels + 8 timestamp).

| Sample Rate | Chunk Duration | `chunk_samples` | 8 slots total |
| --- | --- | ---: | ---: |
| 250 SPS | 100 ms | 25 | 8,800 B |
| 500 SPS | 100 ms | 50 | 17,600 B |
| 1 kSPS | 100 ms | 100 | 35,200 B |
| 4 kSPS | 25 ms | 100 | 35,200 B |
| 16 kSPS | 10 ms | 160 | 56,320 B |
| 16 kSPS | 100 ms | 1600 | **563 KB — not viable** |

**Rule:** `chunk_samples = sample_rate_hz * chunk_duration_ms / 1000` must be chosen so that `ring_buffer_chunks * chunk_samples * 44` fits within available SRAM. At high sample rates, reduce `chunk_duration_ms`.

---

## 3. Ring Buffer API

All functions are `static` (internal to `ads1299.c`). Functions annotated **[ISR]** may only be called from ISR or `post_cb` context. Functions annotated **[task]** may only be called from task context. Functions annotated **[any]** are safe from either.

---

### 3.1 `ads1299_chunkring_init` [task]

```c
static esp_err_t ads1299_chunkring_init(ads1299_chunkring_t *r,
                                         uint32_t             capacity,
                                         uint32_t             chunk_samples);
```

**Purpose:** Allocate and initialise the ring buffer.

**Inputs:**
- `r` — pointer to uninitialised struct
- `capacity` — number of chunk slots; must be power-of-2, >= 2
- `chunk_samples` — samples per chunk; must be > 0

**Step-by-step:**
1. Validate: `r != NULL`, `chunk_samples > 0`, `capacity >= 2`, `capacity & (capacity-1) == 0`. Return `ESP_ERR_INVALID_ARG` otherwise.
2. Allocate `capacity * chunk_samples * sizeof(ads1299_sample_t)` bytes with `malloc`. Return `ESP_ERR_NO_MEM` if NULL.
3. Set `r->buf = <allocated>`, `r->capacity = capacity`, `r->chunk_samples = chunk_samples`, `r->mask = capacity - 1`.
4. Set `r->head = 0`, `r->tail = 0`.
5. Return `ESP_OK`.

**Concurrency:** Must be called before the ISR is armed. No synchronisation required.

---

### 3.2 `ads1299_chunkring_free` [task]

```c
static void ads1299_chunkring_free(ads1299_chunkring_t *r);
```

**Purpose:** Release ring buffer memory.

**Step-by-step:**
1. Call `free(r->buf)`.
2. `memset(r, 0, sizeof(*r))`.

**Concurrency:** Must be called after ISR is disabled and handler task has exited.

---

### 3.3 `ads1299_chunkring_available` [any]

```c
static inline uint32_t ads1299_chunkring_available(const ads1299_chunkring_t *r)
{
    return r->head - r->tail;   /* unsigned arithmetic; wraps correctly */
}
```

**Purpose:** Number of committed, unread chunks.

**Concurrency:** Reading two separate volatile fields is not atomic. The ISR only ever increments `head`; the task only ever increments `tail`. The worst-case race yields a stale (lower) count on the reader side, which is conservative and safe — the caller will check again next iteration.

---

### 3.4 `ads1299_chunkring_free_slots` [any]

```c
static inline uint32_t ads1299_chunkring_free_slots(const ads1299_chunkring_t *r)
{
    return (r->capacity - 1u) - (r->head - r->tail);
}
```

**Purpose:** Number of slots that can be committed before ring is full.

**Note:** `capacity - 1` is the usable maximum (sentinel slot pattern).

---

### 3.5 `ads1299_chunkring_write_ptr` [ISR]

```c
static inline ads1299_sample_t *
ads1299_chunkring_write_ptr(const ads1299_chunkring_t *r, uint32_t sample_idx)
{
    return &r->buf[(r->head & r->mask) * r->chunk_samples + sample_idx];
}
```

**Purpose:** Returns a pointer to sample position `sample_idx` within the current (uncommitted) write slot.

**Usage:** Called in `post_cb` once per sample, with `sample_idx = ctx->sample_count` before incrementing it.

**Concurrency:** Only the ISR writes `head`, and `head` is not modified during an incomplete slot — it advances only at `commit`. This pointer is stable for the duration of the current chunk.

---

### 3.6 `ads1299_chunkring_commit` [ISR]

```c
static inline bool ads1299_chunkring_commit(ads1299_chunkring_t *r)
{
    if (ads1299_chunkring_free_slots(r) == 0) {
        return false;   /* ring full; caller increments overflow_count */
    }
    /* Release fence: all sample writes to buf[] must be visible to the
     * consumer before head is incremented. On Xtensa this emits MEMW. */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    r->head++;
    return true;
}
```

**Purpose:** Publish the current write slot to the consumer.

**On overflow (returns false):**
- `head` is NOT incremented.
- `sample_count` is reset to 0 by the caller.
- The same physical slot (`head & mask`) is overwritten by the next chunk.
- The oldest committed chunk (at `tail`) is untouched — only the newest incoming chunk is dropped.
- `tail` is never modified by the ISR, preserving the SPSC invariant.

**Concurrency:** Only the ISR calls `commit`. The release fence ensures the consumer sees all sample writes to `buf[]` before it observes the incremented `head`.

---

### 3.7 `ads1299_chunkring_read_ptr` [task]

```c
static inline const ads1299_sample_t *
ads1299_chunkring_read_ptr(const ads1299_chunkring_t *r)
{
    /* Acquire fence: read all sample data written before the ISR's
     * release fence in commit(). */
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return &r->buf[(r->tail & r->mask) * r->chunk_samples];
}
```

**Purpose:** Returns a pointer to the oldest committed chunk's sample array.

**Precondition:** `ads1299_chunkring_available() > 0`.

**Lifetime:** The returned pointer is valid until `ads1299_chunkring_release()` is called. The ISR can be writing to a different slot (`head & mask`) concurrently; it will never touch `tail & mask` until the ring has wrapped fully, which requires consuming all existing slots first.

---

### 3.8 `ads1299_chunkring_release` [task]

```c
static inline void ads1299_chunkring_release(ads1299_chunkring_t *r)
{
    /* Release fence: ensure tail increment is visible to the ISR's
     * free_slots() check before the ISR observes the freed space. */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    r->tail++;
}
```

**Purpose:** Mark the oldest committed chunk as consumed; free its slot for reuse.

**Precondition:** `on_chunk()` has returned — the application is done reading the sample data at `tail & mask`.

---

## 4. End-to-End Data Flow

```
Context       Component            Action
──────────    ─────────────────    ──────────────────────────────────────────────
GPIO ISR      drdy_isr_handler     Timestamp capture; spi_busy guard; queue_trans
SPI ISR       spi_post_transfer_cb Parse DMA bytes; write to ring; commit; notify
Task          ads1299_handler_task Drain ret_queue; read ring; build chunk; on_chunk()
User code     on_chunk callback    Process or copy chunk->samples (valid during call only)
```

The DMA buffer (`dma_buf`, 27 bytes) is the only staging area between the ADS1299 hardware and the parsed ring buffer. It is overwritten on every sample; `spi_busy` and the parse-before-clear ordering make this safe.

---

## 5. Stage Implementation

### 5.1 DRDY ISR — `drdy_isr_handler`

**Context:** GPIO interrupt, ISR priority. Must return in bounded time. No blocking calls.

```c
static void IRAM_ATTR drdy_isr_handler(void *arg)
{
    ads1299_t         *dev = (ads1299_t *)arg;
    ads1299_dma_ctx_t *ctx = dev->dma_ctx;

    /* ── Guard ────────────────────────────────────────────────────── */
    if (ctx->spi_busy) {
        /* Previous DMA not yet parsed. This DRDY pulse is dropped.
         * At 250 SPS / 4 MHz SPI, the DMA takes ~54 µs; inter-sample
         * gap is 4000 µs. This path fires only under hardware fault. */
        ctx->dropped_count++;
        return;
    }

    /* ── Timestamp ─────────────────────────────────────────────────── */
    /* Capture at DRDY edge — the most accurate possible timestamp.
     * current_timestamp is read in post_cb. Race is impossible:
     * post_cb (SPI ISR, higher priority) cannot interrupt the GPIO ISR;
     * the next GPIO ISR cannot fire until spi_busy is cleared in post_cb. */
    ctx->current_timestamp = esp_timer_get_time();

    /* ── Arm DMA ───────────────────────────────────────────────────── */
    ctx->spi_busy = true;

    /* trans.rx_buffer = ctx->dma_buf (set once at start_continuous).
     * trans.user      = ctx            (set once at start_continuous).
     * No fields to modify; just re-queue. */
    esp_err_t err = spi_device_queue_trans(dev->spi_handle, &ctx->trans, 0);
    if (err != ESP_OK) {
        /* queue_trans failed (SPI trans_queue full or bus error).
         * Clear spi_busy so the next DRDY can retry. */
        ctx->spi_busy = false;
        ctx->dropped_count++;
    }
}
```

**What this stage does NOT do:**
- Does not parse data.
- Does not touch the ring buffer.
- Does not notify the handler task.
- Does not call any FreeRTOS blocking API.

---

### 5.2 DMA Completion Callback — `spi_post_transfer_cb`

**Context:** Called from the SPI interrupt handler immediately after DMA completes, before the transaction is placed on the SPI driver's `ret_queue`. This is ISR context at SPI interrupt priority — higher than the GPIO ISR. It cannot be preempted by the DRDY ISR.

```c
static void IRAM_ATTR spi_post_transfer_cb(spi_transaction_t *trans)
{
    if (!trans->user) return;   /* guard for non-DMA transactions (SDATAC etc.) */
    ads1299_dma_ctx_t *ctx = (ads1299_dma_ctx_t *)trans->user;

    /* ── 1. Parse DMA bytes into current ring slot ─────────────────── */
    /* dma_buf is stable: spi_busy has been true since the DRDY ISR
     * queued this transaction, preventing any new DMA from starting. */
    ads1299_sample_t *dst =
        ads1299_chunkring_write_ptr(&ctx->ring, ctx->sample_count);
    ads1299_parse_frame(ctx->dma_buf, ctx->current_timestamp, dst);

    /* ── 2. Advance sample position ────────────────────────────────── */
    /* Increment before clearing spi_busy so the next DRDY ISR
     * sees the correct write position if it fires immediately after. */
    ctx->sample_count++;

    /* ── 3. Release DMA buffer ──────────────────────────────────────── */
    /* Cleared AFTER parse. From this point, the next DRDY ISR may
     * set spi_busy and queue a new transaction that overwrites dma_buf. */
    ctx->spi_busy = false;

    /* ── 4. Check chunk completion ──────────────────────────────────── */
    if (ctx->sample_count < ctx->ring.chunk_samples) {
        return;   /* chunk still in progress */
    }

    ctx->sample_count = 0;

    /* ── 5. Commit slot to ring ─────────────────────────────────────── */
    if (!ads1299_chunkring_commit(&ctx->ring)) {
        /* Ring full: newest chunk dropped.
         * Same physical slot will be overwritten by the next chunk.
         * The handler task's tail pointer is untouched. */
        ctx->overflow_count++;
        return;
    }

    /* ── 6. Notify handler task ─────────────────────────────────────── */
    /* eSetValueWithOverwrite: if task has not woken yet and a previous
     * notification is pending, this overwrites it (same value = 1).
     * No notification is lost: the task drains all available chunks
     * in a loop regardless of how many notifications triggered it. */
    BaseType_t higher_woken = pdFALSE;
    xTaskNotifyFromISR(ctx->handler_task,
                       1u,
                       eSetValueWithOverwrite,
                       &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
    /* portYIELD_FROM_ISR sets a deferred flag; the SPI ISR completes
     * normally (puts trans on ret_queue), then the context switch fires
     * at ISR exit. Safe to call from within post_cb on ESP32/Xtensa. */
}
```

**Parsing note:** `ads1299_parse_frame` is called here because:
- The DMA buffer is stable and immediately available.
- Parsing is bounded: 3 status byte copies + 8 iterations of 3-byte signed extension ≈ 30–40 instructions. Sub-microsecond at 240 MHz.
- Parsing here avoids any intermediate copy or additional synchronisation in the handler task.
- The handler task receives pre-parsed samples and can call `on_chunk` directly without transformation.

**What this callback must NOT do:**
- Must not call `spi_device_polling_transmit` or any blocking SPI function.
- Must not call `ESP_LOGI` or any logging function.
- Must not call `vTaskDelay`, `xSemaphoreTake(portMAX_DELAY)`, or any blocking FreeRTOS API.
- Must not call `spi_device_get_trans_result` (same ISR context; would deadlock).
- Must not allocate memory via `malloc`.

---

### 5.3 Handler Task — `ads1299_handler_task`

**Context:** FreeRTOS task at `cfg->task_priority`. Pinned to `cfg->task_core`. Blocking FreeRTOS calls are permitted.

```c
static void ads1299_handler_task(void *arg)
{
    ads1299_t         *dev = (ads1299_t *)arg;
    ads1299_dma_ctx_t *ctx = dev->dma_ctx;

    for (;;) {

        /* ── 1. Block until notified ────────────────────────────────── */
        xTaskNotifyWait(
            0,            /* do not clear bits on entry */
            ULONG_MAX,    /* clear all bits on exit     */
            NULL,         /* notification value unused  */
            portMAX_DELAY
        );

        /* ── 2. Check stop signal ───────────────────────────────────── */
        if (ctx->stop_requested) {
            break;
        }

        /* ── 3. Drain SPI ret_queue ─────────────────────────────────── */
        /* post_cb does all data work. get_trans_result here is purely
         * bookkeeping: it frees slots in the SPI driver's internal
         * ret_queue. Without this drain, ret_queue fills after
         * chunk_samples transactions and new queue_trans calls silently
         * fail (timeout 0 returns immediately), breaking the pipeline. */
        {
            spi_transaction_t *result;
            while (spi_device_get_trans_result(dev->spi_handle,
                                               &result, 0) == ESP_OK) {
                /* Nothing to do. Data was handled in post_cb. */
            }
        }

        /* ── 4. Drain all available complete chunks ─────────────────── */
        /* Loop — do not return to xTaskNotifyWait after each chunk.
         * Multiple chunks may have accumulated while this task was
         * processing on_chunk. Draining here avoids needing one
         * notification per chunk and handles backlog naturally. */
        while (ads1299_chunkring_available(&ctx->ring) > 0) {

            /* ── 4a. Acquire pointer into ring (zero-copy) ─────────── */
            const ads1299_sample_t *samples =
                ads1299_chunkring_read_ptr(&ctx->ring);

            /* ── 4b. Build chunk descriptor ─────────────────────────── */
            ads1299_chunk_t chunk = {
                .samples            = samples,
                .n_samples          = ctx->ring.chunk_samples,
                .first_timestamp_us = samples[0].timestamp_us,
                .last_timestamp_us  = samples[ctx->ring.chunk_samples - 1]
                                      .timestamp_us,
            };

            /* ── 4c. Deliver to user ─────────────────────────────────── */
            /* chunk.samples points directly into the ring buffer.
             * It is valid until ads1299_chunkring_release() is called.
             * The user must copy samples if they need them beyond
             * the duration of this call. */
            ctx->on_chunk(&chunk, ctx->cb_ctx);

            /* ── 4d. Release slot ────────────────────────────────────── */
            /* After release, the ISR may reclaim this slot for a
             * future chunk. Do not access chunk.samples after this. */
            ads1299_chunkring_release(&ctx->ring);
        }

        /* Loop back to xTaskNotifyWait. */
    }

    /* ── Shutdown ────────────────────────────────────────────────────── */
    xSemaphoreGive(ctx->done_sem);
    vTaskDelete(NULL);
}
```

**Partial chunk handling:** The ring buffer only stores committed (complete) chunks. There is no partial chunk visible to the handler task. A chunk in progress lives in `ring.buf[head & mask]` at positions `[0 .. sample_count-1]`, with `head` not yet incremented. The handler task reads only `[tail .. head)`, which contains only complete slots. Partial chunks are invisible by design.

**Backlog drain:** The `while (available > 0)` loop is critical. If `on_chunk` takes longer than one chunk period (e.g. 100 ms at 250 SPS), multiple chunks accumulate. A single notification wakes the task, which drains all of them before sleeping again. No notifications are missed; no additional signalling is needed.

---

## 6. Initialization and Shutdown

### 6.1 `ads1299_start_continuous`

```c
esp_err_t ads1299_start_continuous(ads1299_t                        *dev,
                                    const ads1299_continuous_config_t *cfg)
{
    /* ── Validate ────────────────────────────────────────────────────── */
    if (!dev || !cfg || !cfg->on_chunk)  return ESP_ERR_INVALID_ARG;
    if (!dev->initialized)               return ESP_ERR_INVALID_STATE;
    if (dev->dma_ctx)                    return ESP_ERR_INVALID_STATE; /* already running */

    /* ── Allocate context ────────────────────────────────────────────── */
    ads1299_dma_ctx_t *ctx = calloc(1, sizeof(ads1299_dma_ctx_t));
    if (!ctx) return ESP_ERR_NO_MEM;
    dev->dma_ctx = ctx;

    /* ── Compute sizing ──────────────────────────────────────────────── */
    uint32_t ms = cfg->chunk_duration_ms ? cfg->chunk_duration_ms
                                         : ADS1299_DEFAULT_CHUNK_MS;
    uint32_t chunk_samples = ads1299_sample_rate_to_hz(dev->config.sample_rate)
                             * ms / 1000;
    if (chunk_samples == 0) {
        free(ctx); dev->dma_ctx = NULL;
        return ESP_ERR_INVALID_ARG;
    }

    /* ── Determine ring buffer capacity ─────────────────────────────── */
    uint32_t ring_chunks = cfg->ring_buffer_chunks
                           ? cfg->ring_buffer_chunks
                           : ADS1299_RING_BUF_SLOTS;
    /* Round up to next power of 2 */
    ring_chunks--;
    ring_chunks |= ring_chunks >> 1;
    ring_chunks |= ring_chunks >> 2;
    ring_chunks |= ring_chunks >> 4;
    ring_chunks |= ring_chunks >> 8;
    ring_chunks |= ring_chunks >> 16;
    ring_chunks++;
    if (ring_chunks < 2) ring_chunks = 2;

    /* ── Allocate DMA staging buffer ────────────────────────────────── */
    ctx->dma_buf = heap_caps_malloc(ADS1299_FRAME_SIZE, MALLOC_CAP_DMA);
    if (!ctx->dma_buf) goto fail;

    /* ── Allocate ring buffer ────────────────────────────────────────── */
    esp_err_t err = ads1299_chunkring_init(&ctx->ring, ring_chunks, chunk_samples);
    if (err != ESP_OK) goto fail;

    /* ── Populate context ────────────────────────────────────────────── */
    ctx->sample_count   = 0;
    ctx->spi_busy       = false;
    ctx->stop_requested = false;
    ctx->on_chunk       = cfg->on_chunk;
    ctx->on_error       = cfg->on_error;
    ctx->cb_ctx         = cfg->ctx;
    ctx->dropped_count  = 0;
    ctx->overflow_count = 0;

    ctx->done_sem = xSemaphoreCreateBinary();
    if (!ctx->done_sem) goto fail;

    /* ── Configure SPI transaction (set once; never modified at runtime) */
    memset(&ctx->trans, 0, sizeof(spi_transaction_t));
    ctx->trans.length    = ADS1299_FRAME_SIZE * 8;
    ctx->trans.rxlength  = ADS1299_FRAME_SIZE * 8;
    ctx->trans.tx_buffer = NULL;
    ctx->trans.rx_buffer = ctx->dma_buf;
    ctx->trans.user      = ctx;

    /* ── Re-add SPI device with queue_size = chunk_samples ──────────── */
    /* The device was added in ads1299_init() with queue_size = 1 for
     * polling. For async DMA, queue_size must be >= chunk_samples to
     * prevent the SPI driver's ret_queue from filling between handler
     * task wake-ups and silently dropping queue_trans calls. */
    spi_bus_remove_device(dev->spi_handle);
    dev->spi_handle = NULL;

    spi_device_interface_config_t dev_cfg = {
        .mode             = 1,
        .clock_speed_hz   = 4 * 1000 * 1000,
        .spics_io_num     = dev->config.cs_pin,
        .cs_ena_pretrans  = 2,
        .cs_ena_posttrans = 4,
        .queue_size       = chunk_samples,   /* must hold all in-flight trans */
        .post_cb          = spi_post_transfer_cb,
    };
    err = spi_bus_add_device(dev->config.spi_host, &dev_cfg, &dev->spi_handle);
    if (err != ESP_OK) goto fail;

    /* ── Create handler task ─────────────────────────────────────────── */
    /* Task must exist before ISR is armed so that post_cb has a valid
     * handler_task handle for xTaskNotifyFromISR. */
    BaseType_t ok = xTaskCreatePinnedToCore(
        ads1299_handler_task,
        "ads1299_handler",
        4096,                       /* stack; adjust if on_chunk does heavy work */
        dev,
        cfg->task_priority,
        &ctx->handler_task,
        cfg->task_core
    );
    if (ok != pdPASS) goto fail;

    /* ── Arm DRDY ISR ────────────────────────────────────────────────── */
    /* Armed last: handler_task is guaranteed valid when ISR fires. */
    gpio_install_isr_service(0);    /* no-op if already installed */
    err = gpio_isr_handler_add(dev->config.drdy_pin, drdy_isr_handler, dev);
    if (err != ESP_OK) goto fail;

    return ESP_OK;

fail:
    /* Partial cleanup: free whatever was allocated */
    if (ctx->handler_task) { vTaskDelete(ctx->handler_task); ctx->handler_task = NULL; }
    if (ctx->done_sem)     { vSemaphoreDelete(ctx->done_sem); }
    ads1299_chunkring_free(&ctx->ring);
    if (ctx->dma_buf)      { free(ctx->dma_buf); }
    free(ctx);
    dev->dma_ctx = NULL;
    return ESP_ERR_NO_MEM;
}
```

---

### 6.2 `ads1299_stop_continuous`

```c
esp_err_t ads1299_stop_continuous(ads1299_t *dev)
{
    if (!dev)               return ESP_ERR_INVALID_ARG;
    if (!dev->initialized)  return ESP_ERR_INVALID_STATE;
    if (!dev->dma_ctx)      return ESP_OK;   /* already stopped; clean no-op */

    ads1299_dma_ctx_t *ctx = dev->dma_ctx;

    /* ── 1. Disable DRDY ISR ─────────────────────────────────────────── */
    /* No new DRDY pulses will be processed after this returns. Any ISR
     * that had already entered drdy_isr_handler before this call
     * will complete, but no further ones fire. */
    gpio_isr_handler_remove(dev->config.drdy_pin);

    /* ── 2. Wait for any in-flight DMA to complete ───────────────────── */
    /* Spin on spi_busy. The in-flight transaction completes in < 100 µs
     * at 4 MHz SPI. post_cb will fire, parse, and clear spi_busy. */
    while (ctx->spi_busy) {
        esp_rom_delay_us(10);
    }

    /* ── 3. Signal handler task to exit ─────────────────────────────── */
    ctx->stop_requested = true;
    xTaskNotify(ctx->handler_task, 1u, eSetValueWithOverwrite);

    /* ── 4. Wait for handler task to exit ────────────────────────────── */
    xSemaphoreTake(ctx->done_sem, portMAX_DELAY);

    /* ── 5. Final SPI ret_queue drain ────────────────────────────────── */
    {
        spi_transaction_t *result;
        while (spi_device_get_trans_result(dev->spi_handle,
                                           &result, 0) == ESP_OK) {}
    }

    /* ── 6. Restore SPI device to polling configuration ──────────────── */
    spi_bus_remove_device(dev->spi_handle);
    dev->spi_handle = NULL;

    spi_device_interface_config_t poll_cfg = {
        .mode             = 1,
        .clock_speed_hz   = 4 * 1000 * 1000,
        .spics_io_num     = dev->config.cs_pin,
        .cs_ena_pretrans  = 2,
        .cs_ena_posttrans = 4,
        .queue_size       = 1,
        .post_cb          = NULL,
    };
    spi_bus_add_device(dev->config.spi_host, &poll_cfg, &dev->spi_handle);

    /* ── 7. Free resources ────────────────────────────────────────────── */
    vSemaphoreDelete(ctx->done_sem);
    ads1299_chunkring_free(&ctx->ring);
    free(ctx->dma_buf);
    free(ctx);
    dev->dma_ctx = NULL;

    return ESP_OK;
}
```

---

## 7. Concurrency Model

### 7.1 Producers and Consumers


| Resource | Producer | Consumer |
|----------|----------|----------|
| `dma_buf` | SPI DMA hardware | `post_cb` (parse) |
| `current_timestamp` | DRDY ISR | `post_cb` |
| `spi_busy` | DRDY ISR (set), `post_cb` (clear) | DRDY ISR (read) |
| `ring.buf[head & mask][0..sample_count]` | `post_cb` (write samples) | Handler task (read via `read_ptr`) |
| `ring.head` | `post_cb` (`commit`) | Handler task (`available`, `read_ptr`) |
| `ring.tail` | Handler task (`release`) | `post_cb` (`free_slots`) |
| `sample_count` | `post_cb` | `post_cb` (same context; no race) |
| `dropped_count`, `overflow_count` | `post_cb` / DRDY ISR | Application (read; approximate) |



### 7.2 SPSC Guarantee

This is a **Single-Producer Single-Consumer** design:
- Exactly one writer of `head`: `post_cb` (via `commit`)
- Exactly one writer of `tail`: handler task (via `release`)

No locks are required between producer and consumer. The memory fences in `commit` and `read_ptr` ensure sample writes happen-before `head` is visible to the consumer, and `tail` updates are visible to the ISR when it calls `free_slots`.

### 7.3 Why No Locks

Acquiring a mutex from ISR context (`xSemaphoreTake`) requires `portMAX_DELAY = 0` (non-blocking) which fails when contested, making the ISR unpredictable. Spinlocks (`portENTER_CRITICAL_ISR`) disable interrupts globally on that core, blocking all other ISRs. Both are unacceptable for a real-time acquisition pipeline.

The SPSC ring buffer requires only:
1. `volatile` on `head` and `tail` — prevents compiler caching between accesses
2. Memory fences at `commit` and `read_ptr` / `release` — prevents CPU reordering on Xtensa

### 7.4 `spi_busy` — Not a Ring Buffer Lock

`spi_busy` is not part of the ring buffer concurrency model. It is a separate single-flag guard that serialises DMA transactions. Both the DRDY ISR and `post_cb` access it, but:
- DRDY ISR sets it to `true`
- `post_cb` sets it to `false`
- `post_cb` runs at higher interrupt priority than DRDY ISR and cannot be preempted by it

No fence is needed for `spi_busy` because the operations are naturally ordered by interrupt priority on ESP32/Xtensa.

---

## 8. Overflow and Fault Handling

### 8.1 Ring Buffer Full (`overflow_count`)

**Trigger:** `ads1299_chunkring_commit` returns false (all `capacity - 1` slots occupied).

**Behaviour:**
1. `ctx->sample_count` is reset to 0.
2. `ctx->overflow_count++`.
3. The incoming chunk is discarded. The physical slot at `head & mask` will be overwritten when the next chunk completes.
4. No modification to `tail` or any committed slot.
5. If `ctx->on_error` is set: the ISR cannot call it directly. Overflow is communicated through `overflow_count`, which the application can read at any time. A more elaborate design could post a flag for the handler task to call `on_error` after its drain loop.

**Prevention:** Size `ring_buffer_chunks` so that `on_chunk` processing time < `ring_buffer_chunks * chunk_duration_ms`.

### 8.2 DRDY Pulse Dropped (`dropped_count`)

**Trigger:** `ctx->spi_busy == true` when DRDY ISR fires.

**Behaviour:**
1. `ctx->dropped_count++`.
2. ISR returns immediately. No SPI transaction is queued.
3. The ADS1299 conversion result for this sample is lost permanently.

**At 250 SPS / 4 MHz SPI:** DMA takes ~54 µs; inter-sample gap is 4000 µs. Ratio: 1.4%. This path fires only under hardware fault, bus contention, or at very high sample rates with slow SPI clocks.

**At 16 kSPS / 4 MHz SPI:** DMA takes ~54 µs; inter-sample gap is 62.5 µs. Ratio: 86%. At 16 kSPS, the SPI clock must be high enough (or chunk duration short enough) to complete a transaction before the next DRDY. Minimum SPI frequency: `ADS1299_FRAME_SIZE * 8 * sample_rate_hz = 27 * 8 * 16000 = 3.456 MHz`. At 4 MHz SPI the margin is very tight. Consider 8 MHz for 16 kSPS.

### 8.3 `spi_device_queue_trans` Failure

**Trigger:** The SPI driver's `trans_queue` is full (rare; requires `chunk_samples` transactions already queued without being processed).

**Behaviour in ISR:**
```c
esp_err_t err = spi_device_queue_trans(dev->spi_handle, &ctx->trans, 0);
if (err != ESP_OK) {
    ctx->spi_busy = false;
    ctx->dropped_count++;
}
```
`spi_busy` is cleared so the next DRDY can retry.

**Prevention:** Setting `queue_size = chunk_samples` in `start_continuous` and draining `ret_queue` every handler task wake-up ensures `trans_queue` never fills under normal operation.

### 8.4 Statistics Fields Summary

|
Field
|
Written by
|
Meaning
|
|
---
|
---
|
---
|
|
`dropped_count`
|
DRDY ISR
|
DRDY pulses not converted to DMA transactions
|
|
`overflow_count`
|
`post_cb`
|
Complete chunks discarded because ring was full
|

Both are `volatile uint32_t` and can be read from any context. They are monotonically increasing and never reset by the driver. The application can snapshot and diff them.

---

## 9. Sequence Diagrams

### 9.1 Normal Operation (chunk boundary)

```
DRDY ISR          post_cb           ring_buf          handler_task
    │                 │                 │                    │
    │  [sample 1]     │                 │                    │
    ├─spi_busy=true──►│                 │                    │
    │  queue_trans     │                 │                    │
    │                 ├─parse dma_buf──►│                    │
    │                 │  write_ptr(0)   │                    │
    │                 │  parse_frame    │                    │
    │                 ├─sample_count++=►│  (slot[0] written) │
    │                 ├─spi_busy=false  │                    │
    │                 │                 │                    │
    │  [samples 2..24: same as above, sample_count 1..23]    │
    │                 │                 │                    │
    │  [sample 25]    │                 │                    │
    ├─spi_busy=true──►│                 │                    │
    │  queue_trans     │                 │                    │
    │                 ├─parse dma_buf──►│                    │
    │                 │  write_ptr(24)  │                    │
    │                 │  parse_frame    │                    │
    │                 ├─sample_count=25 │                    │
    │                 ├─spi_busy=false  │                    │
    │                 ├─sample_count==chunk_samples          │
    │                 ├─sample_count=0  │                    │
    │                 ├─commit()───────►├─fence              │
    │                 │                 ├─head++             │
    │                 ├─notify─────────────────────────────►│
    │                 │                 │              wakes │
    │                 │                 │   drain ret_queue  │
    │                 │                 │   get_trans_result │
    │                 │                 │   (x25; drains all)│
    │                 │                 ├─available=1       │
    │                 │                 │   read_ptr()      │
    │                 │                 │   (fence; zero-copy)
    │                 │                 │   build chunk{...} │
    │                 │                 │   on_chunk()───────►
    │                 │                 │           [user code runs]
    │                 │                 │◄──────────────────  │
    │                 │                 │   release()        │
    │                 │                 ├─tail++             │
    │                 │                 │   available=0      │
    │                 │                 │   loop exits       │
    │                 │                 │   xTaskNotifyWait  │
    │                 │                 │                    │
```

### 9.2 Backlog Drain (handler task slow)

```
DRDY ISR          post_cb           ring_buf          handler_task
    │                 │                 │                    │
    │  [chunk 1]      │                 │                    │
    │  ...25 samples  │                 │                    │
    │                 ├─commit─────────►├─head=1             │
    │                 ├─notify─────────────────────────────►│
    │                 │                 │              wakes │
    │                 │                 │ [on_chunk running; task busy]
    │  [chunk 2]      │                 │                    │
    │  ...25 samples  │                 │                    │
    │                 ├─commit─────────►├─head=2             │
    │                 ├─notify──────►  eSetValueWithOverwrite (overwrites, same value)
    │                 │                 │                    │
    │  [chunk 3]      │                 │                    │
    │  ...25 samples  │                 │                    │
    │                 ├─commit─────────►├─head=3             │
    │                 ├─notify──────►  overwritten again     │
    │                 │                 │                    │
    │                 │                 │   on_chunk done   │
    │                 │                 │   release; tail=1  │
    │                 │                 │   available=2      │
    │                 │                 │   [loops; no wait] │
    │                 │                 │   read_ptr(tail=1) │
    │                 │                 │   on_chunk─────────►
    │                 │                 │◄──────────────────  │
    │                 │                 │   release; tail=2  │
    │                 │                 │   available=1      │
    │                 │                 │   [loops]          │
    │                 │                 │   read_ptr(tail=2) │
    │                 │                 │   on_chunk─────────►
    │                 │                 │◄──────────────────  │
    │                 │                 │   release; tail=3  │
    │                 │                 │   available=0      │
    │                 │                 │   loop exits       │
    │                 │                 │   xTaskNotifyWait  │
```

Key point: three chunks accumulated; one notification was sent. The task consumed all three in a single drain loop without needing additional notifications.

### 9.3 Ring Buffer Overflow

```
DRDY ISR          post_cb           ring_buf          handler_task
    │                 │         [head-tail = cap-1 = 7]     │
    │  [chunk N]      │         [ring FULL]                 │
    │  ...25 samples  │                 │                    │
    │                 ├─commit─────────►│                    │
    │                 │   free_slots=0  │                    │
    │                 │   returns false │                    │
    │                 ├─overflow_count++│                    │
    │                 ├─sample_count=0  │   [no notify]      │
    │                 │                 │                    │
    │  [chunk N+1]    │                 │                    │
    │  ...25 samples  │                 │                    │
    │  written into ring.buf[head&mask] │                    │
    │  (same physical slot as chunk N)  │                    │
    │  (chunk N overwritten silently)   │                    │
    │                 │                 │                    │
    │                 │  [meanwhile, handler task eventually runs...]
    │                 │                 │   release; tail++  │
    │                 │                 │   free_slots now 1 │
    │                 ├─commit─────────►├─head++ (chunk N+2) │
    │                 ├─notify─────────────────────────────►│
    │                 │                 │              wakes │
```

Chunk N is lost; chunk N+1 silently overwrites it in the same slot. The application detects loss via `ctx->overflow_count`. The oldest committed chunks (at `tail .. head-1`) are preserved.

---

## 10. Key Invariants Summary

1. **`spi_busy`** is always set before `queue_trans` and cleared after `dma_buf` is parsed.
2. **`dma_buf`** is never overwritten while `spi_busy == true`.
3. **`ring.buf[head & mask][0..N-1]`** is written only by `post_cb`; read only by the handler task (after `commit` and `read_ptr`).
4. **`head`** is incremented only by `post_cb`. **`tail`** is incremented only by the handler task.
5. **`ring.head - ring.tail`** never exceeds `ring.capacity - 1`.
6. **`ctx->trans`** fields are never modified after `start_continuous` initialises them.
7. **`handler_task`** is valid (non-NULL) before the DRDY ISR is armed.
8. **`spi_device_get_trans_result`** is called only from handler task context, never from ISR.
9. **`on_chunk`** is called only from handler task context; `chunk->samples` is valid until `release()`.
10. **`ads1299_stop_continuous`** disables the ISR before waiting for the in-flight DMA to complete, guaranteeing no new writes occur after the function returns.
