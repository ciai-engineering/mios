# MIOS Python Tools

## Requirements

- Python 3.11 or newer
- `pip`, [`uv`](https://docs.astral.sh/uv/), or
  [Conda](https://docs.conda.io/)

## Install with `uv`

From the repository root, create an environment and install all dependencies:

```bash
uv venv
uv pip install -r requirements.txt
```

Activate the environment:

```bash
source .venv/bin/activate
```

## Install with `pip`

From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## Install with Conda

From the repository root, create and activate a Python 3.13 environment:

```bash
conda create --name mios python=3.13 pip -y
conda activate mios
python -m pip install -r requirements.txt
```

To install without activating the environment:

```bash
conda create --name mios python=3.13 pip -y
conda run --name mios python -m pip install -r requirements.txt
```

All Python dependencies, including test dependencies, are listed in
[`requirements.txt`](requirements.txt).

## Run tests

```bash
cd ml_service
pytest
```
