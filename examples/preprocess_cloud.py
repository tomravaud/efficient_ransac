from pathlib import Path

import numpy as np

from efficient_ransac.utils.normals import compute_normals_and_curvature
from efficient_ransac.utils.transforms import center
from efficient_ransac.utils.ply import read_ply, write_ply

# TODO: use click to select the file and set parameters


if "__main__" == __name__:
    data_path = Path("data/")
    filename = "lille_street_small"

    # load the point cloud
    cloud_path = data_path / f"{filename}.ply"
    cloud_ply = read_ply(cloud_path.as_posix())
    cloud = np.vstack((cloud_ply["x"], cloud_ply["y"], cloud_ply["z"])).T

    # center the point cloud
    print("Centering the point cloud...")
    cloud = center(cloud)

    # compute normals and curvature
    print("Computing normals and curvature...")
    normals, curvature = compute_normals_and_curvature(cloud, cloud, radius=0.1)

    # save the modified point cloud
    write_ply(
        (data_path / f"{filename}_preprocessed.ply").as_posix(),
        (cloud, normals.astype(np.float32), curvature.astype(np.float32)),
        ["x", "y", "z", "normal_x", "normal_y", "normal_z", "curvature"],
    )
    print(f"Preprocessed point cloud saved to {data_path / filename}_preprocessed.ply")
