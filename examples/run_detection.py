from pathlib import Path

from efficient_ransac import Detector


if __name__ == "__main__":
    data_path = Path("data/")
    filename = "lille_street_small_preprocessed"

    detector = Detector()
    detector.detect(filepath=data_path / f"{filename}.ply")
