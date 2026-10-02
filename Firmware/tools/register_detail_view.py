#!/usr/bin/env python3
"""Measure how much narrower the detail picture is than the wide one: `vision.secondary.view_scale`.

The detail camera's boxes are published in the wide frame as a centred window 1/scale the size
(perception/detection/view.py), so the one number that matters is the focal-length ratio. It does not
depend on the subject's distance, which is why a still scene and two simultaneous previews are enough:
no board, no motion, nothing for the operator to do. Re-run it when a lens or the detail stream's
sensor mode changes.

On the station (numpy + PIL from the station venv; the previews are visiond's own taps):

    run/station-venv/bin/python Firmware/tools/register_detail_view.py \
        /tmp/ota-stack-$(id -u)/preview.jpg /tmp/ota-stack-$(id -u)/preview_detail.jpg

Method: gradient images (robust to the two sensors' different exposure and white balance), the detail
picture resized to each candidate scale and slid over the wide one by FFT normalised cross-correlation,
coarse then fine, with a small roll search. Where the window lands is printed too, but it is parallax as
much as alignment (the optics are centimetres apart) and is deliberately not used. Point the turret at
texture a few metres away; a close face or a blank wall gives a weak peak (the NCC is printed: below
about 0.4, do not trust the scale).
"""
from __future__ import annotations

import argparse

import numpy as np
from PIL import Image


def gray(path: str) -> np.ndarray:
    return np.asarray(Image.open(path).convert("L"), dtype=np.float64)


def gradient(a: np.ndarray) -> np.ndarray:
    gx = np.zeros_like(a)
    gy = np.zeros_like(a)
    gx[:, 1:-1] = a[:, 2:] - a[:, :-2]
    gy[1:-1] = a[2:] - a[:-2]
    return np.hypot(gx, gy)


def ncc_map(image: np.ndarray, template: np.ndarray) -> np.ndarray:
    H, W = image.shape
    h, w = template.shape
    t = template - template.mean()
    tn = np.sqrt((t * t).sum())
    corr = np.fft.irfft2(np.fft.rfft2(image, s=(H, W)) * np.fft.rfft2(t[::-1, ::-1], s=(H, W)),
                         s=(H, W))[h - 1:, w - 1:]
    ii = np.pad(image, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    ii2 = np.pad(image * image, ((1, 0), (1, 0))).cumsum(0).cumsum(1)

    def box(s):
        return s[h:, w:] - s[:-h, w:] - s[h:, :-w] + s[:-h, :-w]

    S, S2 = box(ii), box(ii2)
    return corr / (tn * np.sqrt(np.maximum(S2 - S * S / (h * w), 1e-9)))


def search(wide: np.ndarray, detail: np.ndarray, scales, rolls=(0.0,)):
    wg = gradient(wide)
    source = Image.fromarray(detail.astype(np.uint8))
    best = None
    for roll in rolls:
        rotated = source.rotate(float(roll), resample=Image.BICUBIC) if roll else source
        for k in scales:
            w, h = int(round(wide.shape[1] / k)), int(round(wide.shape[0] / k))
            template = gradient(np.asarray(rotated.resize((w, h), Image.LANCZOS), dtype=np.float64))
            m = ncc_map(wg, template[2:-2, 2:-2])
            i = np.unravel_index(np.argmax(m), m.shape)
            found = (float(m[i]), float(k), float(roll), i[1] - 2, i[0] - 2, w, h)
            if best is None or found[0] > best[0]:
                best = found
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("wide", help="wide preview JPEG (same aspect ratio as the detail one)")
    parser.add_argument("detail", help="detail preview JPEG, captured at the same moment")
    parser.add_argument("--min-scale", type=float, default=2.0)
    parser.add_argument("--max-scale", type=float, default=10.0)
    args = parser.parse_args()
    wide, detail = gray(args.wide), gray(args.detail)
    coarse = search(wide, detail, np.arange(args.min_scale, args.max_scale + 1e-9, 0.1))
    k = coarse[1]
    ncc, k, roll, x, y, w, h = search(wide, detail, np.arange(k - 0.15, k + 0.151, 0.01),
                                      rolls=np.arange(-2.0, 2.01, 0.5))
    cx, cy = (x + w / 2) / wide.shape[1], (y + h / 2) / wide.shape[0]
    print(f"view_scale {k:.2f}  (NCC {ncc:.2f}, roll {roll:+.1f} deg; the window sits at "
          f"{cx:.3f},{cy:.3f} of the wide frame at this scene's depth -- parallax, not used)")
    return 0 if ncc >= 0.4 else 1


if __name__ == "__main__":
    raise SystemExit(main())
