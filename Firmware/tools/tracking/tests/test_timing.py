"""The timing fit recovers a known imaging delay from synthetic frames: a textured scene shifted by
a known yaw motion (two sines, as the station session), each frame blurred over its exposure and
centred on its stamp + delay, consecutive-frame flow fitted against the encoder angle increments."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import timing  # noqa: E402

DEG = math.pi / 180


def render(delay, exposure, rng):
    h, w = 360, 640  # the station's lores frame (the fit works at half of it)
    base = rng.normal(size=(h, w + 120))
    k2 = np.add.outer(np.fft.fftfreq(h) ** 2, np.fft.fftfreq(w + 120) ** 2)
    base = np.real(np.fft.ifft2(np.fft.fft2(base) * np.exp(-0.5 * k2 / 0.03 ** 2)))
    spectrum = np.fft.fft(base, axis=1)
    kx = np.fft.fftfreq(w + 120)
    scale = 1389 * w / 1920
    # Continuous motion from rest at t = 1 s (no step: the servo cannot jump).
    q = lambda t: (4 * DEG * np.sin(2 * np.pi * 0.35 * (t - 1)) + 0.8 * DEG * np.sin(2 * np.pi * 1.1 * (t - 1))) * (t > 1.0)
    frames, stamps = [], []
    for k in range(int(12 / 0.0333)):
        t0 = 0.2 + k * 0.0333
        # Blur over the exposure: the mean of the scene over the exposure window, centred on t0 + delay.
        acc = np.zeros((h, w))
        for s in np.linspace(-0.5, 0.5, 5):
            acc += np.real(np.fft.ifft(spectrum * np.exp(-2j * np.pi * kx * scale * q(t0 + delay + s * exposure)), axis=1))[:, 60:60 + w]
        frames.append(acc / 5 + rng.normal(scale=0.02, size=(h, w)))
        stamps.append(t0)
    enc_t = np.arange(0, 13, 0.001)
    return np.array(frames), (np.array(stamps) * 1e9).astype(np.int64), (enc_t * 1e9).astype(np.int64), q(enc_t)


def test_energy_fit_recovers_delay():
    rng = np.random.default_rng(5)
    for delay in (-0.0203, 0.0071):
        images, t_ns, enc_ns, enc_q = render(delay, 0.033, rng)
        d, r2 = timing.fit_delay(t_ns, timing.flows(images), enc_ns, enc_q)
        assert r2 > 0.8, r2
        assert abs(d - delay) < 2e-3, (d, delay)


def test_exposure_model_separates_the_exposure_term():
    # Delays at the centre row that follow delta_c = c + k*E give back c and k; the row time comes
    # from the shortest exposure and the disagreement widens the uncertainty.
    centre = lambda e: 0.0097 - 0.8 * e
    s = [{"exposure_s": 0.033, "delta_row0_s": centre(0.033) - 19e-6 * 540, "row_time_s": 19e-6, "row_fit_residual_ms": 2.0, "fails": []},
         {"exposure_s": 0.008, "delta_row0_s": centre(0.008) - 12e-6 * 540, "row_time_s": 12e-6, "row_fit_residual_ms": 1.0, "fails": []}]
    c = timing.combine(s)
    assert c["valid"]
    assert abs(c["exposure_coefficient"] - (-0.8)) < 1e-9 and abs(c["timing"]["exposure_fraction"] - (-0.3)) < 1e-9
    assert abs(c["timing"]["row_time_s"] - 12e-6) < 1e-15
    assert abs(c["timing"]["fixed_offset_s"] - (0.0097 - 12e-6 * 540)) < 1e-9
    assert abs(c["timestamp_uncertainty_s"] - (0.002 + 7e-6 * 270)) < 1e-12
