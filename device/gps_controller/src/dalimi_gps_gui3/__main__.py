import os
import sys


def _setup_videoio_defaults() -> None:
    if sys.platform.startswith("linux"):
        os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_LIST", "V4L2")


_setup_videoio_defaults()

from .ui.main import run  # noqa: E402


if __name__ == "__main__":
    run()
