"""One runnable program per build step. Start them with `python -m candyturret <step>` from host/."""
from __future__ import annotations

import time
from pathlib import Path

import cv2

CAPTURES = Path(__file__).resolve().parents[2] / "captures"

# cv2.waitKeyEx codes for the arrow keys (Windows, then Linux/GTK)
LEFT = {2424832, 65361}
UP = {2490368, 65362}
RIGHT = {2555904, 65363}
DOWN = {2621440, 65364}


def save_capture(img, name: str) -> Path:
    CAPTURES.mkdir(exist_ok=True)
    path = CAPTURES / f"{name}-{time.strftime('%Y%m%d-%H%M%S')}.png"
    cv2.imwrite(str(path), img)
    return path


def show(title: str, img, args, n: int, name: str) -> int:
    """Show a frame (unless headless) and return the key pressed, or -2 when --frames says stop."""
    if args.frames and n >= args.frames:
        print("saved", save_capture(img, name))
        return -2
    if args.headless:
        return -1
    cv2.imshow(title, img)
    key = cv2.waitKey(1) & 0xFF
    return -2 if key in (ord("q"), 27) or window_closed(title) else key


def window_closed(title: str) -> bool:
    try:
        return cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True
