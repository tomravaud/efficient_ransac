from typing import Tuple, Optional

import numpy as np
from sklearn.neighbors import KDTree


def PCA(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Compute PCA on a set of points

    Args:
        points (np.ndarray): Point cloud of shape (N, 3)

    Returns:
        Tuple[np.ndarray, np.ndarray]: Eigenvalues and eigenvectors of the covariance matrix
    """
    # barycenter
    center = np.mean(points, axis=0)

    # center the points
    centered_points = points - center

    # covariance matrix
    cov = np.dot(centered_points.T, centered_points) / points.shape[0]

    # eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    return eigenvalues, eigenvectors


def compute_local_PCA(
    query_points: np.ndarray,
    cloud_points: np.ndarray,
    radius: Optional[float] = None,
    k: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute PCA on the neighborhoods of query_points in cloud_points

    Args:
        query_points (np.ndarray): Points of interest of shape (N, 3)
        cloud_points (np.ndarray): Point cloud of shape (M, 3)
        radius (float): Radius of the neighborhood (in meters)
        k (int): Number of neighbors to consider

    Returns:
        Tuple[np.ndarray, np.ndarray]: Eigenvalues and eigenvectors of the covariance matrix
            for each query point

    Raises:
        ValueError: You must specify either radius or k
    """
    # build a KDTree
    tree = KDTree(cloud_points)

    # find neighbors
    if radius is not None:
        neighbors = tree.query_radius(query_points, r=radius)
    elif k is not None:
        neighbors = tree.query(query_points, k=k, return_distance=False)
    else:
        raise ValueError("You must specify either radius or k")

    all_eigenvalues = np.zeros((cloud_points.shape[0], 3))
    all_eigenvectors = np.zeros((cloud_points.shape[0], 3, 3))

    for i, neighbor in enumerate(neighbors):
        # compute PCA
        eigenvalues, eigenvectors = PCA(cloud_points[neighbor])
        all_eigenvalues[i] = eigenvalues
        all_eigenvectors[i] = eigenvectors

    return all_eigenvalues, all_eigenvectors


def compute_normals_and_curvature(
    query_points: np.ndarray,
    cloud_points: np.ndarray,
    radius: Optional[float] = None,
    k: Optional[int] = None,
) -> np.ndarray:
    """Compute normals and curvature for each query point

    Args:
        query_points (np.ndarray): Points of interest of shape (N, 3)
        cloud_points (np.ndarray): Point cloud of shape (M, 3)
        radius (float): Radius of the neighborhood (in meters)
        k (int): Number of neighbors to consider

    Returns:
        np.ndarray: Normals and curvature for each query point
    """
    all_eigenvalues, all_eigenvectors = compute_local_PCA(
        cloud_points, cloud_points, radius=radius, k=k
    )
    lambda_3, lambda_2, lambda_1 = all_eigenvalues.T

    normals = all_eigenvectors[:, :, 0]
    curvature = lambda_3 / (lambda_1 + lambda_2 + lambda_3 + 1e-6)

    return normals, curvature
