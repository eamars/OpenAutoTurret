---
name: python-venv
description: All Python dependencies must be installed in a project-local virtual environment (venv) — never in the system or user-global Python. The venv must stay out of version control.
whenToUse: Before installing Python packages, running Python scripts or Jupyter notebooks, or setting up any Python tooling.
---

# Python virtual environment (venv) policy

Every Python dependency used for a task must live in a **project-local virtual
environment**. The system / user-global Python must never be modified.

## Rules

1. Never install packages into the system or user-global Python: no bare
   `pip install`, no `pip install --user`, no `sudo pip`.
2. Every Python task uses a project-local venv — create one if none exists,
   reuse it if it does.
3. Run all Python code (scripts, notebooks, CLI tools, linters) with the
   venv's interpreter.
4. The venv is local, machine-specific tooling: keep it out of version
   control.

## Creating a venv

Create one venv per project root (`.venv` is the conventional name):

```bash
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
```

If a venv already exists for the project, do not recreate or delete it —
reuse it and install only what is missing.

## Installing dependencies

```bash
.venv/bin/pip install <package>
```

- Pin versions when the environment must be reproducible elsewhere.
- If the project has a dependency manifest (e.g. a requirements file), keep
  it in sync with what the venv contains so the venv can be recreated on
  another machine.

## Running code

Always invoke the venv interpreter explicitly — do not rely on PATH
activation:

```bash
.venv/bin/python your_script.py
.venv/bin/jupyter lab
```

For an interactive shell, activate it instead: `source .venv/bin/activate`.

## Jupyter kernels

If a notebook kernel is needed, register the venv as a kernel instead of
using a system kernel:

```bash
.venv/bin/python -m ipykernel install --user --name <env-name> --display-name "Python (<env-name> venv)"
```

## Git hygiene

Never commit the venv: no `pyvenv.cfg`, no `site-packages/`, no venv
binaries. Before the first install, confirm the venv directory is covered by
the repository's ignore rules; if it is not, add it.
