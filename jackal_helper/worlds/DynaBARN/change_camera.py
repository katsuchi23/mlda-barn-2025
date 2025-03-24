import os
import xml.etree.ElementTree as ET
current_directory = os.path.dirname(os.path.abspath(__file__))

# scan path directory
scan_path = os.path.join(current_directory)

# sort the scan path list
scan_path_list = [os.path.join(scan_path, f) for f in os.listdir(scan_path)]
print(len(scan_path_list))

new_gui = """
<gui fullscreen="0">
    <camera name="user_camera">
        <pose frame="">-2 4 20 0 1.57 3.14</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
    </camera>
</gui>
"""

for filepath in scan_path_list:
    try:
        if filepath.endswith(".world"):
            print(filepath)
            # open the file and add some text before the </world> tag
            with open(filepath, 'r') as file:
                content = file.read()
            # check if the <gui> tag is already in the file
            if "<gui" not in content:
                content = content.replace("</world>", new_gui + "\n</world>")
            with open(filepath, 'w') as file:
                file.write(content)
    except Exception as e:
        print("Error: ", filepath, e)

print("Done")