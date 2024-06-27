import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
def plot(vertices):
    plt.scatter(vertices[:, 0], vertices[:, 1])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
def get_mjcf_flex(name, vertices, edges, grounds, rgba="1 0 0 0.9"):
    mj = ET.Element("mujoco", model=name)
    extension = ET.SubElement(mj, "extension")
    ET.SubElement(extension, "plugin", plugin="mujoco.elasticity.solid")
    worldbody = ET.SubElement(mj, "worldbody")
    for i, (x, y, z) in enumerate(vertices):
        body = ET.SubElement(worldbody, 'body', name=f"v{i+1}", pos=f"{x} {y} {z + 0.005}")
        ET.SubElement(body, 'inertial', pos="0 0 0", mass="0.01", diaginertia="1.66667e-05 1.66667e-05 1.66667e-05")
        if i not in grounds:
            ET.SubElement(body, 'joint', name=f"v{i+1}_j1", pos="0 0 0", axis="1 0 0", type="slide")
            ET.SubElement(body, 'joint', name=f"v{i+1}_j2", pos="0 0 0", axis="0 1 0", type="slide")
            ET.SubElement(body, 'joint', name=f"v{i+1}_j3", pos="0 0 0", axis="0 0 1", type="slide")
    # Add deformable section
    deformable = ET.SubElement(mj, 'deformable')
    # Add flex objects based on edges
    for body_name, flex_objects in edges.items():
        for flex_name, triangles in flex_objects.items():
            if isinstance(triangles[0], list):
                elements = " ".join(" ".join(map(str, triangle)) for triangle in triangles)
            else:
                elements = " ".join(map(str, triangles))
            flex_vertices = " ".join("0 0 0" for _ in range(flex_name.count("v")))
            flex = ET.SubElement(deformable, 'flex', name=body_name, dim="2", body=flex_name,
                                vertex=flex_vertices, element=elements, rgba=rgba)
    # Add equality section
    equality = ET.SubElement(mj, 'equality')
    for body_name in edges.keys():
        ET.SubElement(equality, 'flex', flex=body_name)
    # Convert the XML tree to a string and print it
    tree = ET.ElementTree(mj)
    ET.indent(tree, space="\t", level=0)
    xml_str = ET.tostring(mj, encoding='unicode')
    return xml_str
if __name__ == '__main__':
    vertices = np.array([
        [0, 0, 50],
        [30, 10, 50],
        [40, -20, 50],
        [30, -50, 50],
        [0, -40, 50],
        [-30, 10, 50],
        [-40, -20, 50],
        [-30, -50, 50],
        [0, -140, 50],
        [0, 100, 50],
    ])/500  # Convert to meters
    grounds = (0, 4)
    edges = {
        "body1": {"v1 v2 v4 v5": [[0, 1, 2], [2, 3, 0]]},
        "body2": {"v2 v3 v4": [0, 1, 2]},
        "body3": {"v4 v9 v5": [0, 1, 2]},
        "body4": {"v5 v9 v8": [0, 1, 2]},
        "body5": {"v6 v7 v8": [0, 1, 2]},
        "body6": {"v1 v6 v8 v5": [[0, 1, 2], [2, 3, 0]]},
        "body7": {"v1 v6 v10": [0, 1, 2]},
        "body8": {"v1 v10 v2": [0, 1, 2]},
    }
    name = "valley_fold"
    xml_str = get_mjcf_flex(name, vertices, edges, grounds)
    with open(f"{name}.xml", "w") as f:
        f.write(xml_str)