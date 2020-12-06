import pandas as pd
import cv2
import numpy as np

from typing import Iterable


def mark_components(img: np.ndarray, pnp: pd.DataFrame, pixel_width: float) -> np.ndarray:
    marked_img = img.copy()
    for i, row in pnp.iterrows():
        px_coords = (int(round(row["Mid X"] / pixel_width)),
                     img.shape[0] - int(round(row["Mid Y"] / pixel_width)))
        marked_img = cv2.circle(
            marked_img,
            px_coords,
            3,
            (0,0,255),
            -1)
    return marked_img


def duplicate_components(row: pd.Series, suffixes: Iterable[str]) -> pd.Series:
    row["Quantity"] *= len(suffixes)
    designators = row["Designator"].split(",")
    new_designators = [d + s for s in suffixes for d in designators]
    row["Designator"] = ",".join(new_designators)
    return row
    
    
def panelize(
    pnp: pd.DataFrame,
    bom: pd.DataFrame,
    board_width: float,
    board_height: float,
    n_rows: int, n_cols: int,
    spacing: float,
    bottom_margin: float=0.0,
    left_margin: float=0.0
) -> pd.DataFrame:
    x_cols = ["Mid X"]
    y_cols = ["Mid Y"]
    pnp_dfs = []
    suffixes = []
    for r in range(n_rows):
        for c in range(n_cols):
            x_offset = left_margin + c * (board_width + spacing)
            y_offset = bottom_margin + r * (board_height + spacing)
            df = pnp.copy()
            df[x_cols] = df[x_cols] + x_offset
            df[y_cols] = df[y_cols] + y_offset
            suffix = f"_{r:02d}_{c:02d}"
            suffixes.append(suffix)
            df["Designator"] = df["Designator"] + suffix
            pnp_dfs.append(df)
    bom_out = bom.copy()
    bom_out = bom_out.apply(lambda r: duplicate_components(r, suffixes), axis=1)
    return pd.concat(pnp_dfs), bom_out