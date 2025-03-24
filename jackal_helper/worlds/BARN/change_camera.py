import os
import xml.etree.ElementTree as ET
current_directory = os.path.dirname(os.path.abspath(__file__))

# scan path directory
scan_path = os.path.join(current_directory)

# sort the scan path list
scan_path_list = [os.path.join(scan_path, f) for f in os.listdir(scan_path)]
print(len(scan_path_list))

new_pose = "-2 4 10 0 1.57 3.14"
for filepath in scan_path_list:
    try:
        if filepath.endswith(".world"):
            print(filepath)
            tree = ET.parse(filepath)
            root = tree.getroot()
            world = root[0]
            gui = world[-1]
            camera = gui[0]
            pose = camera[0]
            pose.text = new_pose
            tree.write(filepath)
            break
    except Exception as e:
        print("Error: ", filepath, e)

print("Done")