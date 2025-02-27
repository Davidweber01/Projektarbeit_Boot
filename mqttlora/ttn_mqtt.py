import paho.mqtt.client as mqtt
import base64
import json
import folium
import webbrowser
import re
import os
import csv
from datetime import datetime

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully to the broker.")
        # Subscribe to the gpsData topic
        client.subscribe("v3/htwgprojektarbeit@htwgtest/devices/test-lora-antenne-v1/up")
        print("Subscribed to topic: v3/htwgprojektarbeit@htwgtest/devices/test-lora-antenne-v1/up")
    else:
        print(f"Connection failed with code {rc}")

def hex_to_string(hex_string):
    """
    Konvertiert einen Hex-String in einen normalen String.

    :param hex_string: Hexadezimale Darstellung eines Strings (als String)
    :return: Entschlüsselter normaler String
    """
    byte_array = bytes.fromhex(hex_string)
    return byte_array.decode('utf-8')

def parse_nmea_gnrmc(nmea_string):
    """
    Parse NMEA GNRMC sentence and extract relevant information.
    
    Format: $GNRMC,hhmmss.ss,A,ddmm.mmmm,a,dddmm.mmmm,a,xx.xx,xxx.xx,ddmmyy,xxx.xx,a*hh
    
    Returns a dictionary with parsed data.
    """
    # Remove any leading/trailing whitespace and split by commas
    parts = nmea_string.strip().split(',')
    
    # Check if this is a valid GNRMC sentence
    if not parts[0].endswith('RMC'):
        raise ValueError(f"Invalid GNRMC format: {nmea_string}")
    
    # Check validity flag
    validity = parts[2]
    if validity != 'A':
        raise ValueError(f"Invalid GPS data (status: {validity})")
    
    # Extract information
    time_utc = parts[1]
    latitude_dm = parts[3]
    lat_direction = parts[4]
    longitude_dm = parts[5]
    lon_direction = parts[6]
    speed_knots = parts[7] if parts[7] else "0.0"
    date = parts[9] if len(parts) > 9 else ""
    
    # Convert latitude from degrees and decimal minutes to decimal degrees
    if latitude_dm and longitude_dm:
        lat_degrees = float(latitude_dm[:2])
        lat_minutes = float(latitude_dm[2:])
        lat_decimal = lat_degrees + (lat_minutes / 60.0)
        if lat_direction == 'S':
            lat_decimal = -lat_decimal
            
        # Convert longitude from degrees and decimal minutes to decimal degrees
        lon_degrees = float(longitude_dm[:3])
        lon_minutes = float(longitude_dm[3:])
        lon_decimal = lon_degrees + (lon_minutes / 60.0)
        if lon_direction == 'W':
            lon_decimal = -lon_decimal
    else:
        raise ValueError("Missing latitude or longitude data")
    
    # Format coordinates for display
    lat_str = f"{lat_degrees}° {lat_minutes}' {lat_direction}"
    lon_str = f"{lon_degrees}° {lon_minutes}' {lon_direction}"
    
    # Format date and time if available
    timestamp = ""
    if date and time_utc:
        day = date[0:2]
        month = date[2:4]
        year = f"20{date[4:6]}"  # Assuming 21st century
        hours = time_utc[0:2]
        minutes = time_utc[2:4]
        seconds = time_utc[4:6]
        timestamp = f"{year}-{month}-{day} {hours}:{minutes}:{seconds}"
    
    return {
        'raw': nmea_string,
        'valid': True,
        'latitude': lat_decimal,
        'longitude': lon_decimal,
        'lat_str': lat_str,
        'lon_str': lon_str,
        'speed_knots': float(speed_knots),
        'timestamp': timestamp
    }

def save_coordinates(parsed_data, raw_hex):
    """
    Save coordinates to a CSV file with timestamp
    """
    filename = "gps_coordinates.csv"
    file_exists = os.path.isfile(filename)
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    try:
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            # Write header if file is new
            if not file_exists:
                writer.writerow(['Timestamp', 'GPS_Timestamp', 'Latitude_Raw', 'Longitude_Raw', 
                               'Latitude_Decimal', 'Longitude_Decimal', 'Speed_Knots', 'Raw_NMEA', 'Raw_Hex'])
            
            # Write data
            writer.writerow([
                timestamp, 
                parsed_data['timestamp'],
                parsed_data['lat_str'], 
                parsed_data['lon_str'], 
                parsed_data['latitude'], 
                parsed_data['longitude'],
                parsed_data['speed_knots'],
                parsed_data['raw'],
                raw_hex
            ])
            print(f"Coordinates saved to {filename}")
    
    except Exception as e:
        print(f"Error saving coordinates: {e}")

def display_map(parsed_data):
    """
    Display a live map with the given coordinates.
    """
    try:
        lat = parsed_data['latitude']
        lon = parsed_data['longitude']
        
        map_ = folium.Map(location=[lat, lon], zoom_start=15)
        popup_text = (f"Latitude: {lat:.6f}, Longitude: {lon:.6f}<br>"
                    f"Speed: {parsed_data['speed_knots']} knots<br>"
                    f"Time: {parsed_data['timestamp']}")
        
        folium.Marker([lat, lon], popup=popup_text).add_to(map_)
        map_file = "live_map.html"
        map_.save(map_file)
        webbrowser.open(f"file://{os.path.abspath(map_file)}")
        print("Map opened in the browser.")
    except Exception as e:
        print(f"Error displaying map: {e}")

def on_message(client, userdata, msg):
    try:
        # Parse the JSON string
        json_string = msg.payload.decode()
        data = json.loads(json_string)

        # Extract the frm_payload field
        frm_payload_base64 = data.get("uplink_message", {}).get("frm_payload", "")
        
        # Decode the Base64 payload
        decoded_payload = base64.b64decode(frm_payload_base64)
        hex_representation = decoded_payload.hex().upper()
        
        # Convert hex to string (NMEA sentence)
        nmea_string = hex_to_string(hex_representation)
        
        # Parse NMEA data
        parsed_data = parse_nmea_gnrmc(nmea_string)
        
        # Log the data
        print(f"Original Base64-Daten: {frm_payload_base64}")
        print(f"Decoded NMEA: {nmea_string}")
        print(f"Hex-Darstellung: {hex_representation}")
        print(f"Breitengrad: {parsed_data['lat_str']} ({parsed_data['latitude']})")
        print(f"Längengrad: {parsed_data['lon_str']} ({parsed_data['longitude']})")
        print(f"Geschwindigkeit: {parsed_data['speed_knots']} Knoten")
        
        # Save coordinates to file
        save_coordinates(parsed_data, hex_representation)
        
        # Display on map
        display_map(parsed_data)
        
    except Exception as e:
        print(f"Error processing message: {e}")

def test_with_examples():
    """
    Test the parsing functionality with the provided examples
    """
    examples = [
        "QOdA/CeAAQDP8vmA"
        "24474E524D432C3137333030362E30302C412C343734312E31323437342C4E2C30303930382E37383230302C452C302E3033362C2C3235303232352C2C2C4136",
        "24474E524D432C3137333030372E30302C412C343734312E31323437342C4E2C30303930382E37383230302C452C302E3036332C2C3235303232352C2C2C4136",
        "24474E524D432C3137333030382E30302C412C343734312E31323437362C4E2C30303930382E37383139392C452C302E3130332C2C3235303232352C2C2C4136",
        "24474E524D432C3137333030392E30302C412C343734312E31323438322C4E2C30303930382E37383230352C452C302E3031302C2C3235303232352C2C2C4136",
        "24474E524D432C3137333031302E30302C412C343734312E31323439302C4E2C30303930382E37383231322C452C302E3035302C2C3235303232352C2C2C4136",
        "24474E524D432C3137333031312E30302C412C343734312E31323439382C4E2C30303930382E37383232302C452C302E3033362C2C3235303232352C2C2C4136",
        "24474E524D432C3137333031322E30302C412C343734312E31323530362C4E2C30303930382E37383232372C452C302E3031372C2C3235303232352C2C2C4136"
    ]
    
    print("\n=== Running tests with example data ===\n")
    
    for i, hex_example in enumerate(examples):
        try:
            print(f"\nTest {i+1}:")
            # Convert hex to string
            nmea_string = hex_to_string(hex_example)
            print(f"NMEA string: {nmea_string}")
            
            # Parse NMEA data
            parsed_data = parse_nmea_gnrmc(nmea_string)
            
            # Print results
            print(f"Valid: {parsed_data['valid']}")
            print(f"Latitude: {parsed_data['lat_str']} ({parsed_data['latitude']})")
            print(f"Longitude: {parsed_data['lon_str']} ({parsed_data['longitude']})")
            print(f"Speed: {parsed_data['speed_knots']} knots")
            if parsed_data['timestamp']:
                print(f"Timestamp: {parsed_data['timestamp']}")
                
            # Optionally uncomment to save each test to CSV
            # save_coordinates(parsed_data, hex_example)
            
            # Optionally uncomment to show map for each test
            # display_map(parsed_data)
            
        except Exception as e:
            print(f"Test {i+1} failed with error: {e}")
    
    print("\n=== Test completed ===\n")

# MQTT broker details
broker_address = "eu2.cloud.thethings.industries"
port = 1883
username = "htwgprojektarbeit@htwgtest"
password = "NNSXS.U3QF5OXYYA35TQ6NGRFKEQZS2R6HKNE6NWJ3I4Y.BV6L2S7NJAYU2LNI4TO5ICLTRZIRYQAKNM4QB3VY3UA6FQW7QPHA"

if __name__ == "__main__":
    # Run tests with example data if requested
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        test_with_examples()
        sys.exit(0)
    
    # Setup the client
    client = mqtt.Client()
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        # Connect to the broker
        client.connect(broker_address, port, keepalive=60)
        # Start the loop to process network events
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nDisconnecting from broker...")
        client.disconnect()
        print("Done!")