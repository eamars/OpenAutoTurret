# Measuring the principal point with the encoder theodolite (round 72)

Objective (c) has been reported "OPEN — needs a physical survey" for thirty rounds. That is half right. The **ChArUco
route** in `tools/calibrate_camera_intrinsics.py` does need a board placed and repositioned by hand across ≥8 views
(`--min-views 8`). But `calibration/camera_intrinsics.yaml` already proves the camera can be commissioned **without a
board** — fx/fy were measured on the live camera by stepping the axis and phase-correlating a high-contrast strip.
The same rig can measure the principal point. This note works the numbers out first, so the procedure is worth
running rather than worth trying.

## Why the principal point shows up in the angular scale

For a pinhole, a bearing ψ off the optical axis lands at `u = cx + fx·tan ψ`. Differentiating with respect to ψ, and
noting a yaw step changes ψ one-for-one:

    du/dψ = fx·sec²ψ = fx·(1 + tan²ψ) = fx + (u − cx)² / fx

**The angular scale in px/deg is a parabola in u whose vertex is cx.** Everything else about the image — texture,
depth, what the target is — cancels. So: measure px/deg at several commanded image positions u, fit a quadratic, and
the vertex is the principal point.

## Does this rig have enough signal to find the vertex?

Using the station's own measured `fx = 1389`, computed just now (`scale = (fx + (u−cx)²/fx)·π/180`):

| u (px) | u − cx | predicted scale | vs centre |
|---|---|---|---|
| 960 | 0 | **24.24 px/deg** | — |
| 1160 | 200 | 24.75 | +2.1% |
| 1360 | 400 | 26.25 | +8.3% |
| 1560 | 600 | 28.77 | **+18.7%** |

Two things follow. First, the model is not speculative at the centre: it predicts **24.24 px/deg** where the file
records **24.22 px/deg** measured by this method — agreement to 0.1%. Second, the curvature at the frame edge,
**+18.7%**, is larger than the method's own worst recorded residual (8.7% yaw out-and-back, 0.4% pitch). The vertex is
therefore identifiable rather than merely expressible, which is the test an instrument has to pass before it is worth
building.

## Why the operator should care about a 100 px error in cx

Frame-exit margin is exactly what (c) exists to compute, and it is computed from the frame edges. If the true cx is
100 px off the 960 convention:

* the **right-edge** bearing is wrong by **±2.9°**, the **left-edge** by **±2.7°** — asymmetrically, in opposite
  directions on the two sides, so it is not a constant that hides inside a calibration offset;
* at the station's 24.2 px/deg that is ~70 px of claimed margin that isn't there, or margin that is claimed missing.

C1 ("target never leaves the frame") is scored against frame edges, so this error lands directly on an acceptance
criterion. That is the case for commissioning cx rather than documenting it as a convention forever.

## What has to be built, in order of cost

1. **`--strip-at-u` for `tools/probe_theodolite.py`.** Today strip mode auto-selects the highest-contrast row or
   column (`cols`/`rows` argmax, around lines 260–267) — it measures wherever the scene happens to be busy, which is
   right for fx/fy and wrong for this fit, since the fit needs *commanded* u values. A band crop at a requested u is
   the whole change.
2. **Runs at several u**, symmetric about the assumed centre so the sign of the error shows up rather than being
   absorbed: at minimum u ≈ 360, 960, 1560.
3. **A quadratic fit** returning the vertex, with the residuals shown.

## Caveats that must survive into the implementation

* **Depression coupling.** The camera looks down at −15.8°, and the intrinsics file records that the projection
  factor is `cos(b)/cos(θ−b)`. For yaw at fixed depression there is a similar coupling, so scale-vs-u is a parabola
  *plus* a depression term. Symmetric left/right strips make the coupling even and the cx term odd, which is the
  cleanest way to separate them; alternatively fit both and report the covariance. Do not fit a bare parabola to
  one-sided data and call the vertex cx.
* **This measures cx, not boresight.** Boresight (where the optical axis points relative to the axis) needs a
  reference at a *known* bearing — a surveyed distant target, or an alignment performed at two poses. The theodolite
  gives differences, not absolutes.
* Nothing here is signed, and the file's own header ("cx/cy are the GEOMETRIC CENTRE BY CONVENTION, not a
  measurement") stays true until a run replaces it.


---

## RETRACTED by measurement on 2026-09-05 (round 74). The vertex is NOT identifiable on this rig.

The reasoning above compared the curvature (+18.7%) to the method's noise (8.7%) and concluded "signal clears noise,
so the vertex is identifiable". That comparison is wrong in kind. cx enters the scale through
`excess = (u−cx)²/fx`, so it is recovered by a **square root of the excess over fx** — which amplifies scale error
enormously wherever the excess is small. The identifiability test is the derivative, not a signal/noise ratio.

Measured today on a freshly homed station (`--strip-at-u`, 1 deg steps, `/tmp/r74_u*.log`), and computed from it:

| band | measured px/deg | cx it implies | what an 8.7% scale error does to cx |
|---|---|---|---|
| u=960 | 26.02 | 584 **or** 1336 | −410 px |
| u=1560 | 29.28 | 927 **or** 2193 | −148 px |
| u=360 | 7.81 | no real root | — (unusable band) |

The direction was right — the edge really does measure a larger scale (29.28 vs 26.02) — and the edge band even lands
within 1.8% of what the model predicted for cx=960. But the two bands **disagree about cx by hundreds of pixels**, and
the sensitivity table settles it: at the most favourable band, this rig's own 8.7% scale residual moves cx by
**148 px**, and reaching ±10 px would need a **0.5%** scale measurement — roughly eighteen times better than the out-
and-back play of the encoder walk. Beating that with a multi-point fit was not attempted: the residual is partly
systematic (mechanical play on reversal), which averaging does not remove.

Conclusions that survive: the curvature is real and measurable in sign; `--strip-at-u` is a working capability;
and the **±2.9° of frame-edge bearing** at stake in a 100 px cx error is still worth fixing. What does not survive is
that this particular theodolite route can fix it. **The board route in `calibrate_camera_intrinsics.py` — ChArUco
across ≥8 views, which needs the board placed and moved by hand — is the only path to cx at useful precision on this
station**, which is what this document said was necessary before I tried to find a cheaper way.

The u=360 band is reported as unusable rather than discarded silently: it read 7.81 px/deg, a third of the base
scale, which no pinhole produces. The estimator returned a confident number for a band without usable texture; "the
tool printed a value" is not evidence that the value means anything, and this time the tool did not even print NaN.
