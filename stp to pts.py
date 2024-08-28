from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopoDS import TopoDS_Shape, topods
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_VERTEX
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Display.SimpleGui import init_display


def load_step_file(file_path):
    """Load a STEP file and return the shape."""
    step_reader = STEPControl_Reader()
    status = step_reader.ReadFile(file_path)
    if status != IFSelect_RetDone:
        raise Exception("Error loading STEP file.")
    step_reader.TransferRoots()
    return step_reader.OneShape()


def extract_points_from_shape(shape, mesh_resolution=0.1):
    """Extract points from a shape using a mesh."""
    points = []

    # Create a mesh on the shape
    mesh = BRepMesh_IncrementalMesh(shape, mesh_resolution)
    mesh.Perform()

    # Extract vertices from the mesh
    exp = TopExp_Explorer(shape, TopAbs_VERTEX)
    while exp.More():
        vertex = topods.Vertex(exp.Current())  # Ensure topods.Vertex is used
        point = BRep_Tool.Pnt(vertex)
        points.append((point.X(), point.Y(), point.Z()))
        exp.Next()

    return points


def save_to_pts(points, output_file):
    """Save points to a PTS file."""
    with open(output_file, 'w') as file:
        file.write(f"{len(points)}\n")
        for pt in points:
            file.write(f"{pt[0]} {pt[1]} {pt[2]}\n")


def main():
    # File paths
    step_file_path = "MP_8F COVER (MACH)_3D file_RD_180404.stp"  # Update to your STEP file path
    output_pts_file_path = "output_file.pts"  # Update to your desired output PTS file path

    # Load the STEP file
    shape = load_step_file(step_file_path)

    # Extract points with finer mesh resolution
    points = extract_points_from_shape(shape, mesh_resolution=1)  # Adjust resolution as needed

    # Save points to PTS file
    save_to_pts(points, output_pts_file_path)

    print(f"PTS file saved as {output_pts_file_path}")

    # Optional: Display the shape (for verification)
    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(shape, update=True)
    start_display()


if __name__ == "__main__":
    main()
