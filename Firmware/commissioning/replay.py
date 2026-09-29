"""Strict synthetic replay interchange for both C++ executables."""
from __future__ import annotations
from pathlib import Path
from .native import CONTROL_FIELDS, CObserver, Simulation


def write_replay(path: Path, control, plant, simulation, refs, initial=(0.,0.)):
    rows=["ADR0022_SYNTHETIC_REPLAY_V2"]
    def row(values):rows.append(" ".join(str(x) for x in values))
    for p in (control,plant):
        n=p.model.n
        row((n,p.model.periodic));row(p.model.q[:n]);row(p.model.z);row(p.model.theta[:7+6*n])
        row(getattr(p.observer,k) for k,_ in CObserver._fields_)
        row(getattr(p,k) for k in CONTROL_FIELDS);row(p.start_total[:6*n]);row(p.start_censored[:6*n])
    row(getattr(simulation,k) for k,_ in Simulation._fields_)
    row((len(refs),*initial))
    for reference in refs:row(reference)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text("\n".join(rows)+"\n",encoding="utf-8")
