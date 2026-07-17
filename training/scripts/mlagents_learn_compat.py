#!/usr/bin/env python3
"""Run ML-Agents with the legacy ONNX exporter required by its 1.15 contract.

ax310 uses a newer CUDA PyTorch whose ``torch.onnx.export`` default is the
dynamo exporter.  That path requires onnxscript and a newer ONNX than the
versions pinned by ML-Agents 1.2.0.dev0.  ML-Agents' exporter was written for
the legacy path, so select it explicitly without changing the locked vision
and protobuf dependency set.
"""

from __future__ import annotations

import atexit
from functools import wraps
import os
import threading

from mlagents.torch_utils import default_device, torch


_torch_onnx_export = torch.onnx.export


@wraps(_torch_onnx_export)
def _legacy_onnx_export(*args, **kwargs):
    kwargs.setdefault("dynamo", False)
    return _torch_onnx_export(*args, **kwargs)


torch.onnx.export = _legacy_onnx_export


# torch.set_default_device() installs a thread-local TorchFunctionMode in the
# ax310 PyTorch build. ML-Agents may create trajectory tensors in its optional
# background trainer thread, so install the selected device mode in every new
# Python thread as well. This also makes old/smoke YAML files with
# ``threaded: true`` safe on CUDA.
_thread_run = threading.Thread.run


@wraps(_thread_run)
def _run_with_torch_device(self, *args, **kwargs):
    torch.set_default_device(default_device())
    return _thread_run(self, *args, **kwargs)


threading.Thread.run = _run_with_torch_device

from mlagents.trainers.learn import main  # noqa: E402


def _install_pid_file() -> None:
    """Publish the trainer PID so dg5f stop never signals forked env workers."""
    pid_file = os.environ.get("DG5F_TRAINER_PID_FILE")
    if not pid_file:
        return

    pid_directory = os.path.dirname(pid_file)
    if pid_directory:
        os.makedirs(pid_directory, exist_ok=True)
    temporary = f"{pid_file}.{os.getpid()}.tmp"
    with open(temporary, "w", encoding="ascii") as file:
        file.write(f"{os.getpid()}\n")
    os.replace(temporary, pid_file)

    def remove_own_pid_file() -> None:
        try:
            with open(pid_file, encoding="ascii") as file:
                owner = file.read().strip()
            if owner == str(os.getpid()):
                os.unlink(pid_file)
        except FileNotFoundError:
            pass

    atexit.register(remove_own_pid_file)


if __name__ == "__main__":
    _install_pid_file()
    main()
