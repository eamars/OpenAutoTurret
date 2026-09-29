"""Bounded current-equivalent model; no extrapolation and no physical defaults."""
from __future__ import annotations
import numpy as np
from .contracts import ModelSpec, Reason, array, require


def basis(nodes, x, *, periodic=False):
    nodes, values = np.asarray(nodes), np.asarray(x, dtype=float)
    require(bool(np.isfinite(values).all()), Reason.DATA_INVALID, "nonfinite coordinate")
    if periodic:
        period = 2 * np.pi
        values = (values - nodes[0]) % period + nodes[0]
        extended = np.r_[nodes, nodes[0] + period]
    else:
        require(bool(np.all((values >= nodes[0] - 1e-10) & (values <= nodes[-1] + 1e-10))),
                Reason.OPERATING_POINT_CHANGED, "coordinate outside model applicability; no extrapolation")
        values, extended = np.clip(values, nodes[0], nodes[-1]), nodes
    left = np.clip(np.searchsorted(extended, values, side="right") - 1, 0, len(extended) - 2)
    weight = (values - extended[left]) / (extended[left + 1] - extended[left])
    out = np.zeros(values.shape + (len(nodes),))
    flat = out.reshape(-1, len(nodes)); indexes = np.arange(flat.shape[0])
    flat[indexes, left.ravel()] = 1 - weight.ravel()
    flat[indexes, ((left + 1) % len(nodes)).ravel()] += weight.ravel()
    return out


def features(spec: ModelSpec, q, z, direction):
    q, z, direction = np.broadcast_arrays(q, z, direction)
    require(bool(np.isin(direction, [-1, 1]).all()), Reason.DATA_INVALID,
            "running load requires a known direction")
    p, s = basis(spec.q_nodes, q, periodic=spec.periodic), basis(spec.z_nodes, z)
    h = np.zeros(q.shape + (2, 3, len(spec.q_nodes)))
    for index, d in enumerate((-1, 1)):
        h[..., index, :, :] = (direction == d)[..., None, None] * s[..., :, None] * p[..., None, :]
    return s, h.reshape(q.shape + (-1,))


def coefficients(spec: ModelSpec, theta, q, z, direction):
    theta = array(theta, (spec.size,), "model parameters")
    s, h = features(spec, q, z, direction)
    return s @ theta[:3], s @ theta[3:6], h @ theta[6:-1]
