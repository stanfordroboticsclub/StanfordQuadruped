import pymeshlab


def calculate_inertial_tag(file_name=None, mass=-1, pr=8, scale_factor=100):
    ms = pymeshlab.MeshSet()

    if file_name is None:
        print('Please put the input file to the same folder as this script and type in the full name of your file.')
        file_name = input()
    ms.load_new_mesh(file_name)

    if mass < 0:
        print('Please type the mass of your object in kg')
        mass = float(input())

    print('Calculating the center of mass')
    geom = ms.get_geometric_measures()
    com = geom['barycenter']

    print('Scaling the mesh')
    ms.compute_matrix_from_scaling_or_normalization(axisx=scale_factor, axisy=scale_factor, axisz=scale_factor)

    print('Generating the convex hull of the mesh')
    ms.generate_convex_hull()  # TODO only if object is not watertight

    print('Calculating intertia tensor')
    geom = ms.get_geometric_measures()
    volume = geom['mesh_volume']
    tensor = geom['inertia_tensor'] / pow(scale_factor, 2) * mass / volume

    intertial_xml = f'<inertial>\n  <origin xyz="{com[0]:.{pr}f} {com[1]:.{pr}f} {com[2]:.{pr}f}"/>\n  <mass value="{mass:.{pr}f}"/>\n  <inertia ixx="{tensor[0, 0]:.{pr}f}" ixy="{tensor[1, 0]:.{pr}f}" ixz="{tensor[2, 0]:.{pr}f}" iyy="{tensor[1, 1]:.{pr}f}" iyz="{tensor[1, 2]:.{pr}f}" izz="{tensor[2, 2]:.{pr}f}"/>\n</inertial>'
    print(intertial_xml)


if __name__ == '__main__':
    calculate_inertial_tag()  # TODO command line arguments
