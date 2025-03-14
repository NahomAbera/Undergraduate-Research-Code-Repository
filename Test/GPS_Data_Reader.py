import serial
import pynmea2
import csv

ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

with open("gps_data.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Latitude", "Longitude", "Fix Quality", "Number of Satellites", "Altitude (m)"])

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("$GPGGA"):
                try:
                    msg = pynmea2.parse(line)
                    writer.writerow([
                        msg.timestamp,
                        msg.latitude,
                        msg.longitude,
                        msg.gps_qual,
                        msg.num_sats,
                        msg.altitude
                    ])
                    print(f"{msg.timestamp}, {msg.latitude}, {msg.longitude}")
                except pynmea2.ParseError:
                    print("Parse error")

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
