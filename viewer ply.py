import open3d as o3d

def load_and_visualize_ply(file_path):
    """Load and visualize a PLY file using Open3D."""
    # Load the PLY file
    pcd = o3d.io.read_point_cloud(file_path)

    # Check if the point cloud has been loaded correctly
    if pcd.is_empty():
        print("Failed to load point cloud.")
        return

    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add the point cloud to the visualizer
    vis.add_geometry(pcd)

    # Optional: Change point size for better visibility
    render_option = vis.get_render_option()
    render_option.point_size = 1.0  # Increase point size (default is 1.0)

    # Update and run visualizer
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.run()

def main():
    # File path
    ply_file_path = "output_file.ply"  # Update to your PLY file path

    # Load and visualize the PLY file
    load_and_visualize_ply(ply_file_path)

if __name__ == "__main__":
    main()
