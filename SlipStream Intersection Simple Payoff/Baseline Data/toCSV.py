import csv
import xml.etree.ElementTree as ET

def xml_to_csv(xml_file, csv_file):
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        
        # Open CSV file for writing
        with open(csv_file, 'w', newline='') as f:
            writer = None
            
            # Loop through all child elements (e.g., <tripinfo>, <lane>, <edge>)
            for child in root:
                data = child.attrib # Get the data as a dictionary
                
                # Create the header (columns) based on the first item found
                if writer is None:
                    writer = csv.DictWriter(f, fieldnames=data.keys())
                    writer.writeheader()
                
                writer.writerow(data)
                
        print(f"Success! Converted {xml_file} -> {csv_file}")
        
    except Exception as e:
        print(f"Error converting {xml_file}: {e}")

# --- Run the conversion ---
xml_to_csv("tripinfo.xml", "tripinfo.csv")