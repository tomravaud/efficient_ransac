from pathlib import Path

from efficient_ransac import Detector


if __name__ == "__main__":
    # viewer = Viewer()
    # viewer.show_cloud(filepath=Path("data/bunny.ply"))

    detector = Detector()
    detector.detect(filepath=Path("data/lille_street_small.ply"))
    # detector.detect(filepath=Path("data/bunny.ply"))
