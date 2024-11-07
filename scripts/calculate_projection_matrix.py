"""
Script adapted from:
https://github.com/Soooooda69/volumetric_drilling/blob/2d938df22b20d69efa8219e466b48ec7d381112a/scripts/camera_projection.py

"""

import numpy as np
import ruamel.yaml

np.set_printoptions(precision=5, suppress=True)


def print_with_fmt(matrix):
    # Format the matrix as a string
    matrix_str = np.array2string(matrix, separator=", ")
    # matrix_str = np.array2string(matrix, separator=', ', formatter={'float_kind':lambda x: f"{x: .5f}"})

    # Adjust the formatting to match the desired output
    matrix_str = (
        matrix_str.replace("[", " [")
        .replace(" [ ", " [")
        .replace("\n ", "\n                      ")
    )

    # Construct the output string
    output_str = (
        "  ## Calculated from calculate_projection_matrix.py\n"
        + "  projection matrix: "
        + matrix_str[1:]
    )
    print(output_str)


def load_yaml(filename):
    yaml = ruamel.yaml.YAML()
    yaml.boolean_representation = ["false", "true"]

    with open(filename, "r") as f:
        params = yaml.load(f)
        f.close()

    camera_params = params["main_camera"]

    return camera_params


def main():

    # camera_params = load_yaml("./simple_scene/ADF/world/world.yaml")
    # near = camera_params["clipping plane"]["near"]
    # far = camera_params["clipping plane"]["far"]
    # width = camera_params["publish image resolution"]["width"]
    # height = camera_params["publish image resolution"]["height"]
    near = 0.001
    far = 10


    ################################
    # Load or hardcode camera matrix
    ################################

    #ZED MINI
    # width = 640
    # height = 360
    # K = [333.00433349609375, 0.0, 319.36297607421875, 0.0, 333.00433349609375, 185.9060821533203, 0.0, 0.0, 1.0]
    # K = np.array(K).reshape(3,3)

    # DVRK endoscope
    width = 1300
    height = 1024
    # K = [1641.829441479134, 0, 634.6603702829163, 0, 1640.859192883227, 464.4736799468848, 0, 0, 1] # raw camera info
    K = [1646.280979764441, 0, 570.1895455154779, 0, 1646.854319884765, 464.5057365925962, 0, 0, 1] # projection matrix
    K = np.array(K).reshape(3, 3) 


    fx = K[0, 0]
    fy = K[1, 1]
    print(f"near {near} far {far} width {width} height {height}")

    #########################
    # Calculate field of view
    #########################

    fov_fx = 2 * np.arctan(0.5 * height / fx)
    fov_fy = 2 * np.arctan(0.5 * height / fy)

    print(f"fov_fx {fov_fx:0.4f} rad, {np.degrees(fov_fx):0.4f} deg")
    print(f"fov_fy {fov_fy:0.4f} rad, {np.degrees(fov_fy):0.4f} deg")
    print(f"fov of 1.2 rad is {np.degrees(1.2):0.4f} deg")

    #########################
    # Calculate field of view
    #########################
    print("\nProjection matrix -> copy-paste output below to ADF file\n")

    p_M = np.zeros([4, 4])
    depth = far - near

    # option 1
    print("  ## option 1")
    p_M[0, 0] = 2 * K[0, 0] / width
    p_M[0, 2] = (width - 2 * K[0, 2]) / width
    p_M[1, 1] = 2 * K[1, 1] / height
    p_M[1, 2] = (-height + 2 * K[1, 2]) / height
    p_M[2, 2] = (-far - near) / depth
    p_M[2, 3] = -2 * (far * near) / depth
    p_M[3, 2] = -1

    # print(np.array2string(p_M, separator=", "))
    print_with_fmt(p_M)

    # option 2
    print("  ## option 2")
    K[0, 2] = width / 2
    K[1, 2] = height / 2

    p_M[0, 0] = 2 * K[0, 0] / width
    p_M[0, 2] = (width - 2 * K[0, 2]) / width
    p_M[1, 1] = 2 * K[1, 1] / height
    p_M[1, 2] = (-height + 2 * K[1, 2]) / height
    p_M[2, 2] = (-far - near) / depth
    p_M[2, 3] = -2 * (far * near) / depth
    p_M[3, 2] = -1

    print_with_fmt(p_M)


if __name__ == "__main__":
    main()
