import serial
import csv
import time
from datetime import datetime

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) #try /dev/ttyUSB* if this don't work 
time.sleep(2)

with open('sensor_data.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Timestamp', 'Soil Moisture Value', 'Soil Moisture (%)', 'Soil Temperature (°C)'])

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split('|')
                if len(parts) == 3:
                    moisture_value = parts[0].split(':')[1].strip()
                    moisture_percent = parts[1].split(':')[1].strip().replace('%', '')
                    temperature_c = parts[2].split(':')[1].strip().replace('°C', '')

                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    csvwriter.writerow([timestamp, moisture_value, moisture_percent, temperature_c])
                    print(f"Logged data at {timestamp}")

                csvfile.flush()
            time.sleep(1)
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print(f"Error: {e}")


ser.close()
