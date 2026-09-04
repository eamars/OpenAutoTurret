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
