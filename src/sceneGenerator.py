import random
import xml.etree.ElementTree as ET
import pybullet as p
import pybullet_data
import time

class SceneGenerator:
    def __init__(self, area_size=5.0):
        self.area_size = area_size
        self.objects = ["cube", "cuboid", "cone", "cylinder", "sphere"]

    def random_position(self, size):
        x = round(random.uniform(-self.area_size / 2 + size, self.area_size / 2 - size), 2)
        y = round(random.uniform(-self.area_size / 2 + size, self.area_size / 2 - size), 2)
        return x, y

    def create_object(self, obj_type, id, base_position=None, color="1 1 1 1"):
        size = round(random.uniform(0.1, 0.5), 2)
        elasticity = round(random.uniform(0.3, 1.0), 2)
        plasticity = round(1 - elasticity, 2)
        x, y = base_position if base_position else self.random_position(size)
        z = size / 2 + 0.01  # Ensure object is slightly above the ground plane

        model = ET.Element("model", name=f"{obj_type}_{id}")
        pose = ET.SubElement(model, "pose")
        pose.text = f"{x} {y} {z} 0 0 0"

        link = ET.SubElement(model, "link", name="link")
        
        # Collision element
        collision = ET.SubElement(link, "collision", name="collision")
        collision_geometry = ET.SubElement(collision, "geometry")
        
        # Visual element with color
        visual = ET.SubElement(link, "visual", name="visual")
        visual_geometry = ET.SubElement(visual, "geometry")
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = color
        ET.SubElement(material, "diffuse").text = color

        if obj_type in ["cube", "cuboid"]:
            size_x, size_y, size_z = (size, size, size) if obj_type == "cube" else (size, size * 1.5, size / 2)
            box = ET.SubElement(collision_geometry, "box")
            visual_box = ET.SubElement(visual_geometry, "box")
            ET.SubElement(box, "size").text = f"{size_x} {size_y} {size_z}"
            ET.SubElement(visual_box, "size").text = f"{size_x} {size_y} {size_z}"
        elif obj_type == "sphere":
            sphere = ET.SubElement(collision_geometry, "sphere")
            visual_sphere = ET.SubElement(visual_geometry, "sphere")
            ET.SubElement(sphere, "radius").text = str(size)
            ET.SubElement(visual_sphere, "radius").text = str(size)
        elif obj_type == "cylinder":
            cylinder = ET.SubElement(collision_geometry, "cylinder")
            visual_cylinder = ET.SubElement(visual_geometry, "cylinder")
            ET.SubElement(cylinder, "radius").text = str(size / 2)
            ET.SubElement(cylinder, "length").text = str(size)
            ET.SubElement(visual_cylinder, "radius").text = str(size / 2)
            ET.SubElement(visual_cylinder, "length").text = str(size)
        elif obj_type == "cone":
            cone = ET.SubElement(collision_geometry, "cone")
            visual_cone = ET.SubElement(visual_geometry, "cone")
            ET.SubElement(cone, "radius").text = str(size / 2)
            ET.SubElement(cone, "length").text = str(size)
            ET.SubElement(visual_cone, "radius").text = str(size / 2)
            ET.SubElement(visual_cone, "length").text = str(size)

        # Surface properties for elasticity and plasticity
        surface = ET.SubElement(collision, "surface")
        contact = ET.SubElement(surface, "contact")
        ode = ET.SubElement(contact, "ode")
        ET.SubElement(ode, "soft_erp").text = str(elasticity)
        ET.SubElement(ode, "soft_cfm").text = str(plasticity)
        
        return model

    def generate_scene_pair(self, num_objects, num_linked, initial_filename="initial_scene.sdf", final_filename="final_scene.sdf"):
        initial_sdf = ET.Element("sdf", version="1.6")
        initial_world = ET.SubElement(initial_sdf, "world", name="default")

        final_sdf = ET.Element("sdf", version="1.6")
        final_world = ET.SubElement(final_sdf, "world", name="default")

        for i in range(num_objects):
            obj_type = random.choice(self.objects)
            base_position = None
            is_linked = i < num_linked

            # Create initial object in green
            obj_initial = self.create_object(obj_type, i, base_position, color="0 1 0 1")  # Green
            initial_world.append(obj_initial)

            initial_pose = obj_initial.find("pose").text
            initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw = map(float, initial_pose.split())

            if random.random() > 0.5:
                new_x = round(random.uniform(-self.area_size / 2 + 0.5, self.area_size / 2 - 0.5), 2)
                new_y = round(random.uniform(-self.area_size / 2 + 0.5, self.area_size / 2 - 0.5), 2)
                new_z = 0.5 if is_linked else random.uniform(0.1, 0.5)
                new_orientation = (0, 0, random.uniform(0, 3.14))
                pose_text = f"{new_x} {new_y} {new_z} {new_orientation[0]} {new_orientation[1]} {new_orientation[2]}"
            else:
                new_x, new_y = initial_x, initial_y
                new_z, new_orientation = initial_z, (initial_roll, initial_pitch, initial_yaw)
                pose_text = f"{new_x} {new_y} {new_z} {new_orientation[0]} {new_orientation[1]} {new_orientation[2]}"

            # Create final object in blue
            obj_final = self.create_object(obj_type, i, (new_x, new_y), color="0 0 1 1")  # Blue
            obj_final.find("pose").text = pose_text
            final_world.append(obj_final)

        initial_tree = ET.ElementTree(initial_sdf)
        initial_tree.write(initial_filename, encoding="utf-8", xml_declaration=True)

        final_tree = ET.ElementTree(final_sdf)
        final_tree.write(final_filename, encoding="utf-8", xml_declaration=True)

# Set Camera View
def set_camera():
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

# Load Scene Function
def load_scene(file_path, offset_x=0):
    objects = p.loadSDF(file_path)
    for obj in objects:
        pos, orn = p.getBasePositionAndOrientation(obj)
        new_pos = (pos[0] + offset_x, pos[1], pos[2])
        p.resetBasePositionAndOrientation(obj, new_pos, orn)
    return objects

# Main function
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.81)

    set_camera()
    load_scene("initial_scene.sdf", offset_x=-3)  # Initial scene on the left
    load_scene("final_scene.sdf", offset_x=3)     # Final scene on the right

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1. / 240.)
    
    print("Press Ctrl+C to exit.")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
