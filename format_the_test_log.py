import json
import logging
from typing import List, Dict
from datetime import datetime, timedelta
from pymongo import MongoClient
USE_TIME_AS_STRING = False
UPLOAD_MONGODB = True
def parse_log_to_upload_format(log_line: str) -> Dict:
    
    
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
    
    # Convert timestamp to datetime object
    recorded_time = datetime.strptime(data['timestamp'], '%Y-%m-%d %H:%M:%S')
    recorded_time += timedelta(hours=7)
    received_time = datetime.now()
    if USE_TIME_AS_STRING:
        recorded_time = recorded_time.strftime('%Y-%m-%d %H:%M:%S')
        received_time = received_time.strftime('%Y-%m-%d %H:%M:%S')
    upload_data = {
        "recorded_time": recorded_time,
        "received_time": received_time,
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

def process_log_file(input_filename: str, output_filename: str) -> List[Dict]:
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
            if not UPLOAD_MONGODB:
                json.dump(formatted_entries, json_file, indent=2)
            
        logging.info(f"Successfully processed {len(formatted_entries)} entries to {output_filename}")
            
    except Exception as e:
        logging.error(f"Error processing file: {e}")
        raise
    return formatted_entries
def upload_to_mongodb(entries: List[Dict], 
                     connection_string: str,
                     db_name: str,
                     collection_name: str) -> None:
    client = MongoClient(connection_string)
    db = client[db_name]
    collection = db[collection_name]
    
    try:
        result = collection.insert_many(entries)
        logging.info(f"Inserted {len(result.inserted_ids)} documents")
    except Exception as e:
        logging.error(f"Error uploading to MongoDB: {e}")
        raise
    finally:
        client.close()

def main():
    logging.basicConfig(level=logging.INFO)
    input_file = "test_log.txt"
    output_file = "processed_logs.json"

    mongo_connection = "" # INSERT CONNECTION STRING
    database = "test"
    collection = "test"

    entries = process_log_file(input_file, output_file)
    if UPLOAD_MONGODB:
        upload_to_mongodb(entries, mongo_connection, database, collection)

if __name__ == "__main__":
    main()
