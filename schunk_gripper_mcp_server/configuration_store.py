"""Persistence for MCP-owned gripper configurations."""

import json
from pathlib import Path
from typing import Any

SERVER_DIRECTORY = Path(__file__).resolve().parent.parent
DEFAULT_CONFIGURATION_PATH = SERVER_DIRECTORY / "configuration.json"
CONFIGURATION_PATH_FILE = SERVER_DIRECTORY / ".configuration_path"


def resolve_configuration_path(location: str = "") -> Path:
    if location:
        return Path(location).expanduser().resolve()
    try:
        remembered_location = CONFIGURATION_PATH_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        remembered_location = ""
    return Path(remembered_location) if remembered_location else DEFAULT_CONFIGURATION_PATH


def save_configuration_data(configuration: list[dict[str, Any]], location: str = "") -> str:
    path = resolve_configuration_path(location)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(configuration, indent=2), encoding="utf-8")
    CONFIGURATION_PATH_FILE.write_text(str(path), encoding="utf-8")
    return str(path)


def load_configuration_data(location: str = "") -> tuple[list[Any] | None, Path, str | None]:
    path = resolve_configuration_path(location)
    try:
        content = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None, path, f"No saved configuration at {path}."
    except (OSError, json.JSONDecodeError) as error:
        return None, path, f"Cannot read {path}: {error}"
    if not isinstance(content, list):
        return None, path, f"Unexpected layout in {path}."
    return content, path, None