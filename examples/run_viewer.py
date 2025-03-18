from pathlib import Path
import datetime
import shutil
import click

from efficient_ransac import Detector, Viewer


@click.command()
@click.argument("input_path", type=click.Path(exists=True, path_type=Path))

def main(
    input_path: Path,
) -> None:

    # visualize the detection
    viewer = Viewer()
    viewer.show_cloud(
        filepath=input_path,
    )


if __name__ == "__main__":
    main()
