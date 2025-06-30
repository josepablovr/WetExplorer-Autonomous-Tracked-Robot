import numpy as np
import open3d as o3d

def generate_cylinder_surface(radius, height, n_points):
    """
    Generate a point cloud representing the surface of a cylinder.

    Parameters:
        radius (float): Radius of the cylinder.
        height (float): Height of the cylinder.
        n_points (int): Number of points to generate.

    Returns:
        numpy.ndarray: A (n_points, 3) array of 3D points representing the cylinder surface.
    """
    # Generate random angles and heights for the points
    angles = np.random.uniform(0, 2*np.pi, n_points)
    heights = np.random.uniform(0, height, n_points)

    # Convert cylindrical coordinates to Cartesian coordinates
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    z = heights

    # Stack the coordinates to form the point cloud
    points = np.column_stack((x, y, z))
    return points

def generate_cone_surface(initial_radius, max_radius, height, n_points):
    """
    Generate a point cloud representing the surface of a cone.

    Parameters:
        initial_radius (float): Radius of the base of the cone.
        max_radius (float): Maximum radius of the cone.
        height (float): Height of the cone.
        n_points (int): Number of points to generate.

    Returns:
        numpy.ndarray: A (n_points, 3) array of 3D points representing the cone surface.
    """
    # Generate random angles and heights for the points
    angles = np.random.uniform(0, 2 * np.pi, n_points)
    heights = np.random.uniform(0, height, n_points)

    # Linearly interpolate the radius based on height
    radii = initial_radius + (max_radius - initial_radius) * (heights / height)

    # Convert cylindrical coordinates to Cartesian coordinates
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)
    z = heights

    # Stack the coordinates to form the point cloud
    points = np.column_stack((x, y, z))
    return points

def visualize_point_cloud(points):
    """
    Visualize a point cloud using Open3D.

    Parameters:
        points (numpy.ndarray): A (n_points, 3) array of 3D points.
    """
    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()

    # Assign the points to the PointCloud object
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])

def save_point_cloud(points, filename):
    """
    Save a point cloud to a PLY file.

    Parameters:
        points (numpy.ndarray): A (n_points, 3) array of 3D points.
        filename (str): Path to the output PLY file.
    """
    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()

    # Assign the points to the PointCloud object
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Write the point cloud to a PLY file
    o3d.io.write_point_cloud(filename, point_cloud)

if __name__ == "__main__":
    # Parameters for the cylinder
    cylinder_radius = 0.19/2  # Cylinder radius 0.20
    cylinder_height = 0.045 # Cylinder height
    n_cylinder_points = 5000  # Number of points on the cylinder surface

    # Parameters for the cone
    cone_initial_radius = cylinder_radius  # Cone base radius matches cylinder top radius
    cone_max_radius = 0.35/2  # Cone apex radius
    cone_height = 0.0447446 # Cone height
    n_cone_points = 10000  # Number of points on the cone surface

    # Generate the cylinder surface point cloud
    cylinder_points = generate_cylinder_surface(cylinder_radius, cylinder_height, n_cylinder_points)

    # Generate the cone surface point cloud
    cone_points = generate_cone_surface(cone_initial_radius, cone_max_radius, cone_height, n_cone_points)

    # Shift the cone points to sit on top of the cylinder
    cone_points[:, 2] += cylinder_height

    cylinder2_points = generate_cylinder_surface(cone_max_radius, cylinder_height/2, 1000)
    cylinder2_points[:, 2] += cylinder_height/2 + cone_height
    # Combine the cylinder and cone point clouds
    combined_points = np.vstack((cylinder_points, cone_points))
    #combined_points = np.vstack((combined_points, cylinder2_points))
    combined_points*=10
    # Save the combined point cloud to a PLY file
    save_point_cloud(combined_points, "/ros2_ws/src/wetexplorer/wetexplorer_vision/wetexplorer_vision/CAD_v3.ply")

    # Visualize the combined point cloud
    visualize_point_cloud(combined_points)
