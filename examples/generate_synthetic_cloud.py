from pathlib import Path

import numpy as np

from efficient_ransac.utils.synthetic_shapes import generate_random_shapes
from efficient_ransac.utils.ply import write_ply


if __name__ == "__main__":
    print("Generating a synthetic point cloud...")
    cloud = generate_random_shapes(
        num_planes=3,
        num_spheres=3,
        num_cylinders=3,
        density=1000,
    )

    out_path = Path("data/synthetic_cloud.ply")
    write_ply(
        out_path.as_posix(),
        (cloud.astype(np.float32),),
        ["x", "y", "z"],
    )
    print(f"Synthetic point cloud saved to {out_path}")
