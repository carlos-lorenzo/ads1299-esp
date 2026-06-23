from __future__ import annotations

from pathlib import Path
import os
import shutil
import subprocess


DOCS_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = DOCS_ROOT.parent
DOXYFILE = DOCS_ROOT / "Doxyfile"
DOXYGEN_XML = DOCS_ROOT / "xml"

project = "ADS1299 ESP-IDF Driver"
author = "Carlos Lorenzo"
release = "0.1.0"
version = release
master_doc = "index"
languages = ["en"]
idf_targets = []
project_slug = "ads1299-esp"
pdf_file_prefix = "ads1299-esp"
pdf_file = ""
html_zip = ""
versions_url = ""
project_homepage = "https://github.com/carlos-lorenzo/ads1299-esp"
download_url = ""
latest_branch_name = "main"
idf_target_title_dict = {}

extensions = [
    "breathe",
    "sphinx_copybutton",
]

breathe_projects = {"ads1299": str(DOXYGEN_XML)}
breathe_default_project = "ads1299"

html_theme = "sphinx_idf_theme"
html_static_path = ["../_static"]
templates_path = ["../_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

github_repo = "carlos-lorenzo/ads1299-esp"
html_context = {
    "github_user": "carlos-lorenzo",
    "github_repo": "ads1299-esp",
}


def run_doxygen(_: object) -> None:
    if shutil.which("doxygen") is None:
        raise RuntimeError(
            "Doxygen is required to build the API reference. "
            "Install doxygen locally or use the Read the Docs build image."
        )

    result = subprocess.run(
        ["doxygen", str(DOXYFILE.name)],
        cwd=DOCS_ROOT,
        check=False,
        stderr=subprocess.PIPE,
        text=True,
    )
    build_dir = Path(os.environ.get("BUILDDIR", DOCS_ROOT / "_build"))
    build_dir.mkdir(parents=True, exist_ok=True)
    (build_dir / "doxygen-warning-log.txt").write_text(result.stderr, encoding="utf-8")
    result.check_returncode()


def setup(app: object) -> None:
    app.add_config_value("config_dir", None, "env")
    app.add_config_value("docs_to_build", None, "env")
    app.add_config_value("doxyfile_dir", None, "env")
    app.add_config_value("idf_target", None, "env")
    app.add_config_value("idf_targets", None, "env")
    app.add_config_value("idf_target_title_dict", idf_target_title_dict, "env")
    app.add_config_value("pdf_file", pdf_file, "html")
    app.add_config_value("project_path", None, "env")
    app.connect("builder-inited", run_doxygen)
