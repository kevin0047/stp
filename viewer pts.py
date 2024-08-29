import numpy as np  # pip install numpy
import open3d as o3d  # pip install open3d
import matplotlib.pyplot as plt  # pip install matplotlib

def load_pts_file(file_path):
    """Load points from a PTS file."""
    with open(file_path, 'r') as file:
        lines = file.readlines()

    num_points = int(lines[0].strip())
    points = []

    for line in lines[1:num_points + 1]:
        x, y, z = map(float, line.strip().split())
        points.append([x, y, z])

    return np.array(points)


def visualize_points(points):
    """Visualize the points using Open3D with height-based color mapping and lighting."""
    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Normalize z values to range [0, 1] for color mapping
    z_min = points[:, 2].min()
    z_max = points[:, 2].max()
    z_normalized = (points[:, 2] - z_min) / (z_max - z_min)

    # Apply a color map (e.g., viridis) based on the normalized z values
    colors = plt.get_cmap('viridis')(z_normalized)[:, :3]  # Get RGB values from the colormap
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)

    # Change point size
    render_option = vis.get_render_option()
    render_option.point_size = 1.0  # Increase point size (default is 1.0)

    # Enable lighting effect
    render_option.light_on = True

    # Update and run visualizer
    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()
    vis.run()


def main():
    # File path
    pts_file_path = "output_file.pts"  # Update to your PTS file path

    # Load points from the PTS file
    points = load_pts_file(pts_file_path)

    # Visualize the points
    visualize_points(points)


if __name__ == "__main__":
    main()
