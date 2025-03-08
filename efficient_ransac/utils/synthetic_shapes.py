import numpy as np


def generate_plane(
    density: int = 500,
    size_range: tuple = (1, 3),
    center_range: tuple = (-2, 2),
) -> np.ndarray:
    """Generate a point cloud representing a plane

    Args:
        density (int, optional): Density of the point cloud in points per square unit
            of the plane. Defaults to 500.
        size_range (tuple, optional): Range of the size of the plane in meters
            (width and height). Defaults to (1, 3).
        center_range (tuple, optional): Range of the position of the center of the
            plane in meters (x, y, z). Defaults to (-2, 2).

    Returns:
        np.ndarray: Generated point cloud
    """
    width = np.random.uniform(*size_range)
    height = np.random.uniform(*size_range)
    center = np.random.uniform(*center_range, size=3)

    # random normal vector
    normal = np.random.randn(3)
    normal /= np.linalg.norm(normal)

    # find two orthogonal vectors in the plane
    u = np.cross(normal, [1, 0, 0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(normal, [0, 1, 0])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    # generate points in the plane
    num_points = int(density * width * height)
    u_vals = np.random.uniform(-width / 2, width / 2, num_points)
    v_vals = np.random.uniform(-height / 2, height / 2, num_points)

    points = center + np.outer(u_vals, u) + np.outer(v_vals, v)
    return points


def generate_sphere(
    density: int = 500,
    radius_range: tuple = (0.5, 1.5),
    center_range: tuple = (-2, 2),
) -> np.ndarray:
    """Generate a point cloud representing a sphere

    Args:
        density (int, optional): Density of the point cloud in points per square unit
            of the sphere surface. Defaults to 500.
        radius_range (tuple, optional): Range of the radius of the sphere.
            Defaults to (0.5, 1.5).
        center_range (tuple, optional): Range of the position of the center of the
            sphere in meters (x, y, z). Defaults to (-2, 2).

    Returns:
        np.ndarray: Generated point cloud
    """
    radius = np.random.uniform(*radius_range)
    center = np.random.uniform(*center_range, size=3)

    # approximate the number of points based on surface area
    surface_area = 4 * np.pi * radius**2
    num_points = int(density * surface_area)

    # generate random points on the unit sphere
    cloud = np.random.randn(num_points, 3)
    cloud /= np.linalg.norm(cloud, axis=1)[:, None]

    return radius * cloud + center


def generate_cylinder(
    density: int = 500,
    radius_range: tuple = (0.5, 1.5),
    height_range: tuple = (1, 3),
    center_range: tuple = (-2, 2),
) -> np.ndarray:
    """Generate a point cloud representing a cylinder

    Args:
        density (int, optional): Density of the point cloud in points per square unit
            of the cylinder surface. Defaults to 500.
        radius_range (tuple, optional): Range of the radius of the cylinder.
            Defaults to (0.5, 1.5).
        height_range (tuple, optional): Range of the height of the cylinder.
            Defaults to (1, 3).
        center_range (tuple, optional): Range of the position of the center of the
            cylinder in meters (x, y, z). Defaults to (-2, 2).

    Returns:
        np.ndarray: Generated point cloud
    """
    radius = np.random.uniform(*radius_range)
    height = np.random.uniform(*height_range)
    center = np.random.uniform(*center_range, size=3)

    # approximate the number of points based on surface area
    surface_area = 2 * np.pi * radius * height
    num_points = int(density * surface_area)

    # sample points on the lateral surface
    theta = np.random.uniform(0, 2 * np.pi, num_points)
    z = np.random.uniform(-height / 2, height / 2, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)

    # random orientation
    axis = np.random.randn(3)
    axis /= np.linalg.norm(axis)

    # default cylinder direction
    default_axis = np.array([0, 0, 1])

    # compute rotation matrix using Rodrigues' formula
    v = np.cross(default_axis, axis)
    s = np.linalg.norm(v)
    c = np.dot(default_axis, axis)
    Vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    if s != 0:
        R = np.eye(3) + Vx + (Vx @ Vx) * ((1 - c) / (s**2))
    else:
        R = np.eye(3)

    return np.column_stack((x, y, z)) @ R.T + center


def generate_random_shapes(
    num_planes: int = 1,
    num_spheres: int = 1,
    num_cylinders: int = 1,
    density: int = 500,
) -> np.ndarray:
    """Generate a point cloud composed of random shapes

    Args:
        num_planes (int, optional): Number of planes to generate. Defaults to 1.
        num_spheres (int, optional): Number of spheres to generate. Defaults to 1.
        num_cylinders (int, optional): Number of cylinders to generate. Defaults to 1.
        density (int, optional): Density of the point cloud in points per square unit
            of the bounding box of the shapes. Defaults to 500.

    Returns:
        np.ndarray: Generated point cloud
    """
    point_clouds = []

    for _ in range(num_planes):
        point_clouds.append(generate_plane(density))

    for _ in range(num_spheres):
        point_clouds.append(generate_sphere(density))

    for _ in range(num_cylinders):
        point_clouds.append(generate_cylinder(density))

    return np.vstack(point_clouds)
