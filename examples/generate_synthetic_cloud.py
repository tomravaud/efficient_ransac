from pathlib import Path

import numpy as np
import click

from efficient_ransac.utils.synthetic_shapes import generate_random_shapes
from efficient_ransac.utils.ply import write_ply


@click.command()
@click.argument("output_path", type=click.Path(path_type=Path))
@click.option("--num_planes", default=3, help="Number of planes")
@click.option("--num_spheres", default=3, help="Number of spheres")
@click.option("--num_cylinders", default=3, help="Number of cylinders")
@click.option("--density", default=1000, help="Density of the point cloud")
def main(
    output_path: Path,
    num_planes: int,
    num_spheres: int,
    num_cylinders: int,
    density: int,
) -> None:
    """
    Generate a synthetic point cloud composed of random planes, spheres, and cylinders.
    """
    print("[INFO] Generating a synthetic point cloud...")
    cloud = generate_random_shapes(
        num_planes=num_planes,
        num_spheres=num_spheres,
        num_cylinders=num_cylinders,
        density=density,
    )

    write_ply(
        output_path.as_posix(),
        (cloud.astype(np.float32),),
        ["x", "y", "z"],
    )
    print(f"[INFO] Synthetic point cloud saved to {output_path}")


if __name__ == "__main__":
    main()
