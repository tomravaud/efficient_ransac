import numpy as np


def centroid(points: np.ndarray) -> np.ndarray:
    """Compute the centroid of a point cloud

    Args:
        points (np.ndarray): Point cloud

    Returns:
        np.ndarray: Centroid
    """
    return np.mean(points, axis=0)


def shifting(points: np.ndarray, shift: np.ndarray) -> np.ndarray:
    """Shift a point cloud by a vector

    Args:
        points (np.ndarray): Point cloud to shift
        shift (np.ndarray): Shift vector

    Returns:
        np.ndarray: Shifted point cloud
    """
    return points + shift


def scaling(points: np.ndarray, factor: float) -> np.ndarray:
    """Scale a point cloud by a factor

    Args:
        points (np.ndarray): Point cloud to scale
        factor (float): Scaling factor

    Returns:
        np.ndarray: Scaled point cloud
    """
    return points * factor


def center(points: np.ndarray) -> np.ndarray:
    """Center a point cloud

    Args:
        points (np.ndarray): Point cloud

    Returns:
        np.ndarray: Centered point cloud
    """
    return shifting(points, -centroid(points))
