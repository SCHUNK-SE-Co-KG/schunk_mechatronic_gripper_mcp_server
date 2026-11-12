# Copyright 2026 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

"""Persistence for MCP-owned gripper configurations."""

import json
from pathlib import Path
from typing import Any

from platformdirs import user_config_path

CONFIGURATION_DIRECTORY = user_config_path("schunk-gripper-mcp-server", appauthor=False, roaming=True)
DEFAULT_CONFIGURATION_PATH = CONFIGURATION_DIRECTORY / "configuration.json"
CONFIGURATION_PATH_FILE = CONFIGURATION_DIRECTORY / ".configuration_path"


def resolve_configuration_path(location: str = "") -> Path:
    if location:
        return Path(location).expanduser().resolve()
    try:
        remembered_location = CONFIGURATION_PATH_FILE.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        remembered_location = ""
    return Path(remembered_location) if remembered_location else DEFAULT_CONFIGURATION_PATH


def save_configuration_data(configuration: list[dict[str, Any]], location: str = "") -> str:
    path = resolve_configuration_path(location)
    CONFIGURATION_DIRECTORY.mkdir(parents=True, exist_ok=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(configuration, indent=2), encoding="utf-8")
    CONFIGURATION_PATH_FILE.write_text(str(path), encoding="utf-8")
    return str(path)


def load_configuration_data(location: str = "") -> tuple[list[Any] | None, Path, str | None]:
    path = DEFAULT_CONFIGURATION_PATH
    try:
        path = resolve_configuration_path(location)
        content = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None, path, f"No saved configuration at {path}."
    except (OSError, json.JSONDecodeError) as error:
        return None, path, f"Cannot read {path}: {error}"
    if not isinstance(content, list):
        return None, path, f"Unexpected layout in {path}."
    return content, path, None