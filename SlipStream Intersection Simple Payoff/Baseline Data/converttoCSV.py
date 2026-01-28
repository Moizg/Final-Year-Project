import csv
import xml.etree.ElementTree as ET

def convert_queue_xml_to_csv(xml_file, csv_file):
    print(f"Converting {xml_file} to {csv_file}...")
    
    try:
        # Parse the XML file
        tree = ET.parse(xml_file)
        root = tree.getroot()
        
        # Prepare CSV file
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Define the CSV Header
            # We explicitly list the columns we want to ensure order
            header = ['timestep', 'lane_id', 'queueing_time', 'queueing_length', 'queueing_length_experimental']
            writer.writerow(header)
            
            # 1. Loop through each <data> block (representing a specific time)
            for data_block in root.findall('data'):
                current_time = data_block.get('timestep')
                
                # 2. Find the <lanes> container inside <data>
                lanes_container = data_block.find('lanes')
                
                if lanes_container is not None:
                    # 3. Loop through each <lane> inside <lanes>
                    for lane in lanes_container.findall('lane'):
                        # Get attributes
                        lane_id = lane.get('id')
                        q_time = lane.get('queueing_time')
                        q_len = lane.get('queueing_length')
                        q_len_exp = lane.get('queueing_length_experimental')
                        
                        # Write the row: [Time, ID, Metrics...]
                        writer.writerow([current_time, lane_id, q_time, q_len, q_len_exp])
                        
        print("Success! Conversion complete.")

    except FileNotFoundError:
        print(f"Error: The file '{xml_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

# --- EXECUTE THE FUNCTION ---
# Make sure the filename matches your actual output file name
convert_queue_xml_to_csv("output_queues.xml", "output_queues.csv")