import zipfile
import xml.etree.ElementTree as ET
import os
import shutil

# Specify the path to the GeoGebra .gbd file
gbd_path = "C:/Users/eli-o/Downloads/gbdpointextract/geogebra-export.ggb"
output_file = "points.txt"

# Create a temporary directory to extract the .gbd file
temp_dir = "temp_geogebra"
os.makedirs(temp_dir, exist_ok=True)

# Extract the .gbd file to access the contained XML files
with zipfile.ZipFile(gbd_path, 'r') as zip_ref:
    zip_ref.extractall(temp_dir)

# Prepare a list to store points
points_list = []

# Iterate through the extracted files and parse XML files to find points
for root_dir, _, files in os.walk(temp_dir):
    for file in files:
        if file.endswith('.xml'):
            file_path = os.path.join(root_dir, file)
            tree = ET.parse(file_path)
            root = tree.getroot()
            for element in root.iter('element'):
                if element.attrib.get('type') == 'point':
                    coords = element.find('coords')
                    if coords is not None:
                        try:
                            x = float(coords.attrib.get('x', 'nan'))
                            y = float(coords.attrib.get('y', 'nan'))
                            # Only add valid points
                            if not (x == 'nan' or y == 'nan'):
                                points_list.append((x, y))
                        except (ValueError, TypeError):
                            # Handle any conversion errors
                            continue

# Write the points to a TXT file in MATLAB vector format
with open(output_file, "w") as file:
    file.write("points = [\n")
    for point in points_list:
        file.write(f"    {point[0]}, {point[1]};\n")
    file.write("];\n")

# Clean up the temporary directory
shutil.rmtree(temp_dir)

print(f"Points successfully written to {output_file}")
