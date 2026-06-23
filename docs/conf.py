import subprocess
import os

# -- Run Doxygen before Sphinx builds --
subprocess.call("doxygen Doxyfile", shell=True, cwd=os.path.dirname(__file__))

# -- Project info --
project = "ADS1299 ESP-IDF Driver"
author = "Carlos"
release = "0.1.0"

# -- Extensions --
extensions = [
    "breathe",
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
]

# -- Breathe config --
breathe_projects = {"ads1299_esp": "./xml"}   # path to Doxygen XML output
breathe_default_project = "ads1299_esp"

# -- Theme --
html_theme = "sphinx_rtd_theme"

# -- Paths --
templates_path = ["_templates"]
html_static_path = ["_static"]
exclude_patterns = ["_build"]
