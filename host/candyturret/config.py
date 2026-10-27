"""Load config/turret.toml and list the TEMP placeholders in it."""
from __future__ import annotations

try:
    import tomllib
except ModuleNotFoundError:          # Python 3.10, e.g. JetPack 6 on the Jetson
    import tomli as tomllib
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "config" / "turret.toml"


def _ns(d: dict) -> SimpleNamespace:
    return SimpleNamespace(**{k: _ns(v) if isinstance(v, dict) else v for k, v in d.items()})


def load(path: str | Path | None = None) -> SimpleNamespace:
    with open(Path(path) if path else DEFAULT_CONFIG, "rb") as f:
        return _ns(tomllib.load(f))


def temp_values(path: str | Path | None = None) -> list[tuple[str, str, str]]:
    """(section.key, value, note) for every setting marked '# TEMP'."""
    text = (Path(path) if path else DEFAULT_CONFIG).read_text(encoding="utf-8")
    out, section = [], ""
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("[") and s.endswith("]"):
            section = s[1:-1]
        elif "# TEMP" in s and "=" in s and not s.startswith("#"):
            body, note = s.split("# TEMP", 1)
            key, value = body.split("=", 1)
            out.append((f"{section}.{key.strip()}", value.strip(), note.lstrip(":").strip()))
    return out
