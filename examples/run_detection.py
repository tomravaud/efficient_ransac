from pathlib import Path
import datetime
import shutil
import click

from efficient_ransac import Detector, Viewer


@click.command()
@click.argument("input_path", type=click.Path(exists=True, path_type=Path))
@click.option(
    "--output_path", default=None, type=click.Path(path_type=Path), help="Output path"
)
@click.option(
    "--config_path",
    default=Path("configs/config.yaml"),
    type=click.Path(path_type=Path),
    help="Path to the configuration file",
)
def main(
    input_path: Path,
    output_path: Path | None,
    config_path: Path,
) -> None:
    """
    Detect planes, spheres, and cylinders in a point cloud using Efficient RANSAC.
    Save and visualize the detection.
    """
    if output_path is None:
        output_path = (
            Path("output/")
            / datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            / f"{input_path.stem}.ply"
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    shutil.copy(config_path, output_path.parent / config_path.name)
    

    # Efficient RANSAC detection
    detector = Detector(config_path)
    detector.detect(
        input_path=input_path,
        output_path=output_path,
    )

    # visualize the detection
    viewer = Viewer()
    viewer.show_cloud(
        filepath=output_path,
    )


if __name__ == "__main__":
    main()
