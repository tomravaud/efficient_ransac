from pathlib import Path

import numpy as np
import click

from efficient_ransac.utils.normals import compute_normals_and_curvature
from efficient_ransac.utils.transforms import center
from efficient_ransac.utils.ply import read_ply, write_ply


@click.command()
@click.argument("input_path", type=click.Path(exists=True, path_type=Path))
@click.option("--centering/--no-centering", default=True, help="Center the point cloud")
@click.option(
    "--normals/--no-normals", default=True, help="Compute normals and curvature"
)
@click.option("--radius", default=0.1, help="Radius for normal estimation")
@click.option(
    "--num_neighbors",
    default=None,
    type=int,
    help="Number of neighbors for normal estimation",
)
@click.option(
    "--decimation", default=None, type=int, help="Decimate the point cloud by a factor"
)
@click.option(
    "--output_path", default=None, type=click.Path(path_type=Path), help="Output path"
)
def main(
    input_path: Path,
    centering: bool,
    normals: bool,
    radius: float | None,
    num_neighbors: int | None,
    decimation: int | None,
    output_path: Path | None,
) -> None:
    """
    Preprocess a point cloud by centering, computing normals, and decimating.
    """
    data = {}
    filename = input_path.stem

    # load the point cloud
    cloud_ply = read_ply(input_path.as_posix())
    cloud = np.vstack((cloud_ply["x"], cloud_ply["y"], cloud_ply["z"])).T
    cloud = cloud.astype(np.float32)
    data["x"], data["y"], data["z"] = cloud.T

    # center the point cloud
    if centering:
        print("[INFO] Centering the point cloud...")
        cloud = center(cloud)

    # compute normals and curvature
    if normals:
        print("[INFO] Computing normals and curvature...")
        if radius is None and num_neighbors is None:
            raise ValueError("Either radius or num_neighbors must be provided")
        cloud_normals, curvature = compute_normals_and_curvature(
            cloud,
            cloud,
            radius=radius,
            k=num_neighbors,
        )
        cloud_normals = cloud_normals.astype(np.float32)
        curvature = curvature.astype(np.float32)
        data["normal_x"], data["normal_y"], data["normal_z"] = cloud_normals.T
        data["curvature"] = curvature

    # decimate the point cloud
    if decimation is not None and decimation > 0:
        print("[INFO] Decimating the point cloud...")
        cloud = cloud[::decimation]
        if normals:
            cloud_normals = cloud_normals[::decimation]
            curvature = curvature[::decimation]

    if output_path is None:
        output_path = input_path.parent / f"{filename}_preprocessed.ply"

    # save the modified point cloud
    write_ply(
        output_path.as_posix(),
        [*data.values()],
        [*data.keys()],
    )
    print(f"[INFO] Preprocessed point cloud saved to {output_path}")


if __name__ == "__main__":
    main()
