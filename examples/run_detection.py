from pathlib import Path
import datetime

from efficient_ransac import Detector, Viewer


if __name__ == "__main__":
    data_path = Path("data/")
    filename = "synthetic_cloud_preprocessed"
    # filename = "lille_street_small_preprocessed"
    config_path = Path("configs/config.yaml")

    out_path = Path("output/") / datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_path.mkdir(parents=True, exist_ok=True)

    # Efficient RANSAC detection
    detector = Detector(config_path)
    detector.detect(
        input_path=data_path / f"{filename}.ply",
        output_path=out_path / f"{filename}_detection.ply",
    )

    # Visualize the detection
    viewer = Viewer()
    viewer.show_cloud(
        filepath=out_path / f"{filename}_detection.ply",
    )
