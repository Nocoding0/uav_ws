"""Small persistent JSONL log for one marker-follow node run."""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional
from uuid import uuid4


class MarkerRunLog:
    def __init__(self, directory: str) -> None:
        folder = Path(directory).expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        name = "marker_follow_{}_{}_{}.jsonl".format(
            datetime.now().strftime("%Y%m%d_%H%M%S"), os.getpid(), uuid4().hex[:8]
        )
        self.path = str(folder / name)
        self._file = open(self.path, "x", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()
        self.error: Optional[str] = None

    def record(self, event: str, stamp: float, **fields: Any) -> bool:
        row: Dict[str, Any] = {"event": event, "stamp": stamp, **fields}
        with self._lock:
            if self.error is not None:
                return False
            if self._file.closed:
                return False
            try:
                line = json.dumps(row, ensure_ascii=False, sort_keys=True,
                                  allow_nan=False)
                self._file.write(line + "\n")
                self._file.flush()
            except (OSError, TypeError, ValueError) as exc:
                self.error = str(exc)
                return False
        return True

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()
