# Recorded execution environment

## System prerequisites

Before creating the Python environment, provide `git`, `curl`, GNU
`sha256sum`, GNU `tar` with Zstandard support, the `zstd` executable, and
Python 3.12. Release downloads use anonymous public HTTPS and do not require
GitHub CLI, a GitHub account, or an authentication token.

## Replay environment

The immutable Final-Test protocol recorded CPython 3.12.3 on Linux x86_64,
NumPy 1.26.4, Pillow 10.2.0, PyYAML 6.0.1, PyTorch 2.10.0+cu128,
torchvision 0.25.0+cu128, CUDA build 12.8, and cuDNN 91002. The historical
GPU was an NVIDIA GeForce RTX 3080 Laptop GPU (compute capability 8.6).

For the closest replay, use Python 3.12 and a CUDA 12.8-compatible driver:

```bash
python3.12 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-replay.txt
```

CPU replay is supported by the stateless evaluator and is useful for an
independent check, but floating-point details and runtime will differ from
the recorded CUDA environment. The current workstation environment was not
used to generate this lock file because several packages were upgraded after
the historical attempt.
