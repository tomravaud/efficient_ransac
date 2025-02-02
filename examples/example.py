from pathlib import Path

from efficient_ransac import Viewer


if __name__ == "__main__":
    viewer = Viewer()
    viewer.show_cloud(filepath=Path("data/bunny.ply"))
