from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopoDS import topods
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.IFSelect import IFSelect_RetDone
import numpy as np
import open3d as o3d

def load_step_file(file_path):
    """Load a STEP file and return the shape."""
    step_reader = STEPControl_Reader()
    status = step_reader.ReadFile(file_path)
    if status != IFSelect_RetDone:
        raise Exception("Error loading STEP file.")
    step_reader.TransferRoots()
    return step_reader.OneShape()

def extract_points_from_shape(shape, mesh_resolution=0.1, u_resolution=10, v_resolution=10):
    """Extract points from a shape using a mesh and additional surface sampling."""
    points = []

    # Create a mesh on the shape with a very fine resolution
    mesh = BRepMesh_IncrementalMesh(shape, mesh_resolution)
    mesh.Perform()

    # Extract vertices from the mesh
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = topods.Face(exp.Current())
        surf_adaptor = BRepAdaptor_Surface(face)
        surf_handle = surf_adaptor.Surface()  # Obtain the Geom_Surface object

        u_min, u_max = surf_adaptor.FirstUParameter(), surf_adaptor.LastUParameter()
        v_min, v_max = surf_adaptor.FirstVParameter(), surf_adaptor.LastVParameter()

        u_step = (u_max - u_min) / u_resolution
        v_step = (v_max - v_min) / v_resolution

        for i in range(u_resolution + 1):
            for j in range(v_resolution + 1):
                u = u_min + i * u_step
                v = v_min + j * v_step

                # Properly create a handle to the surface
                surf_handle = surf_adaptor.Surface().Surface()
                props = GeomLProp_SLProps(surf_handle, 1, 0.01)  # 기본 생성자를 사용하여 속성을 초기화합니다.
                props.SetParameters(u, v)

                if props.IsNormalDefined():
                    pnt = props.Value()
                    points.append((pnt.X(), pnt.Y(), pnt.Z()))

        exp.Next()

    return points

def save_to_ply(points, output_file):
    """Save points to a PLY file."""
    # Create Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))

    # Save to PLY file
    o3d.io.write_point_cloud(output_file, pcd)

def main():
    # File paths
    step_file_path = "MP_8F COVER (MACH)_3D file_RD_180404.stp"  # Update to your STEP file path
    output_ply_file_path = "output_file.ply"  # Update to your desired PLY file path

    # Load the STEP file
    shape = load_step_file(step_file_path)

    # Extract points with fine resolution
    points = extract_points_from_shape(shape, mesh_resolution=0.001, u_resolution=200, v_resolution=200)  # Increased resolution

    # Save points to PLY file
    save_to_ply(points, output_ply_file_path)

    print(f"PLY file saved as {output_ply_file_path}")

if __name__ == "__main__":
    main()
