from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from conf_common import *  # noqa: F401,F403

language = "en"
copyright = "2026, Carlos Lorenzo"
pdf_title = "ADS1299 ESP-IDF Driver"
