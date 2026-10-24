"""Isolated synthetic inputs for the existing production refiner replay."""
from pathlib import Path
import csv
import numpy as np
from PIL import Image

root = Path(__file__).resolve().parent
images = root / 'vision_images'
images.mkdir(parents=True, exist_ok=True)
cases = [(0.45, 0), (0.10, 0), (0.10, 28), (0.08, 0),
         (0.08, 30), (0.12, 24), (0.10, 20), (0.10, 25)]
y, x = np.mgrid[:480, :640]
with (root / 'vision_diagnostics.tsv').open('w', newline='', encoding='utf-8') as f:
    out = csv.writer(f, delimiter='\t')
    out.writerow(['frame_id', 'before_fusion_x_px', 'before_fusion_y_px'])
    for i, (slope, move) in enumerate(cases, 1):
        center = 320.0 + move
        dx = x - center
        stripe = 230.0 + np.where(dx < 0, slope, -slope) * dx + np.where(dx < 0, 4.0, -4.0)
        gray = np.clip(np.rint(14.0 + 215.0 * np.exp(-(y - stripe) ** 2 / 3.38)), 0, 255).astype(np.uint8)
        Image.fromarray(gray).save(images / f'sample_{i}_frame_{i}_source.png')
        out.writerow([i, center, 230.0])
with (root / 'cases.tsv').open('w', newline='', encoding='utf-8') as f:
    out = csv.writer(f, delimiter='\t')
    out.writerow(['frame', 'left_slope', 'right_slope', 'shift_x', 'gap', 'old_intercept_residual'])
    for i, (slope, move) in enumerate(cases, 1):
        out.writerow([i, slope, -slope, move, -8.0, slope * move])
