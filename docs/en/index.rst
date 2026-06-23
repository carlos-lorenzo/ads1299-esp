ADS1299 ESP-IDF Driver
======================

The ADS1299 ESP-IDF driver provides a C API for configuring a Texas Instruments
ADS1299 analog front-end over SPI and acquiring EEG or biosignal samples from an
ESP-IDF application.

Installation
------------

Add the component to an ESP-IDF project:

.. code-block:: console

   idf.py add-dependency "carlos-lorenzo/ads1299-esp^0.1.0"

The application must initialize the SPI bus before creating and initializing the
ADS1299 device handle. The driver adds and removes its SPI device internally.

Quick Start
-----------

.. code-block:: c

   #include "ads1299.h"

   ads1299_config_t config = {
       .spi_host = SPI2_HOST,
       .cs_pin = GPIO_NUM_5,
       .drdy_pin = GPIO_NUM_4,
       .reset_pin = GPIO_NUM_16,
       .start_pin = GPIO_NUM_17,
       .sample_rate = ADS1299_DR_250SPS,
   };

   ads1299_t dev = ads1299_create(&config);
   ESP_ERROR_CHECK(ads1299_init(&dev));

Contents
--------

.. toctree::
   :maxdepth: 2

   api-reference
