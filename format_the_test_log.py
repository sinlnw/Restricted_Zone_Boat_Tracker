import json
import logging
from typing import List, Dict


def parse_log_to_upload_format(log_line: str) -> Dict:
    from datetime import datetime
    
    # Parse log line
    data = {}
    parts = log_line.split()
    for part in parts:
        if '=' in part:
            key, value = part.split('=')
            if key == 'timestamp':
                # Get full timestamp (both date and time)
                next_part = parts[parts.index(part) + 1]
                value = value + " " + next_part
            data[key] = value
    
    upload_data = {
        "recorded_time": data['timestamp'],  # Now contains both date and time
        "received_time": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        "device": 1,
        "lat": float(data['lat'])/1e2,
        "lon": float(data['long'])/1e2,
        "vbat": float(data['vbat']),
        "quality": int(data['quality']),
        "satellites": int(data['satellites']),
        "temperature": int(data['raw_temp']),
        "ttf": float(data['last_heard']),
        "rssi": -70
    }
    
    return upload_data

def process_log_file(input_filename: str, output_filename: str) -> None:
    formatted_entries = []
    
    try:
        # Read and process log file
        with open(input_filename, 'r') as file:
            for line in file:
                if line.strip():
                    entry = parse_log_to_upload_format(line.strip())
                    formatted_entries.append(entry)
        
        # Write to JSON file
        with open(output_filename, 'w') as json_file:
            json.dump(formatted_entries, json_file, indent=2)
            
        logging.info(f"Successfully processed {len(formatted_entries)} entries to {output_filename}")
            
    except Exception as e:
        logging.error(f"Error processing file: {e}")
        raise

def main():
    logging.basicConfig(level=logging.INFO)
    input_file = "test_log.txt"
    output_file = "processed_logs.json"
    process_log_file(input_file, output_file)

if __name__ == "__main__":
    main()