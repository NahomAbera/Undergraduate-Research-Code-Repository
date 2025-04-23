from flask import Flask, render_template, request, redirect, url_for, send_from_directory
import threading
import serial
import pynmea2
import csv
import time
from datetime import datetime
import os

app = Flask(__name__)
is_logging = False
log_thread = None
user_config = {}

def read_data():
    global is_logging, user_config
    gps_port = '/dev/ttyACM1'
    sensor_port = '/dev/ttyACM0'
    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"data_{timestamp_str}.csv"
    filepath = os.path.join("data", filename)

    with open(filepath, 'w', newline='') as csvfile:
        headers = ['Timestamp']
        if user_config.get('gps'):
            headers += ['Latitude', 'Longitude', 'Fix Quality', 'Number of Satellites', 'Altitude']
        if user_config.get('moisture_value'):
            headers += ['Soil Moisture Value']
        if user_config.get('moisture_percent'):
            headers += ['Soil Moisture (%)']
        if user_config.get('temperature'):
            headers += ['Soil Temperature (°C)']
        writer = csv.writer(csvfile)
        writer.writerow(headers)

        try:
            gps_ser = serial.Serial(gps_port, baudrate=9600, timeout=1)
        except:
            gps_ser = None

        sensor_ser = serial.Serial(sensor_port, 9600, timeout=1)
        time.sleep(2)

        while is_logging:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            row = [timestamp]

            gps_data = ['-99.9999999'] * 5
            if user_config.get('gps') and gps_ser:
                try:
                    line = gps_ser.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("$GPGGA"):
                        msg = pynmea2.parse(line)
                        gps_data = [msg.latitude, msg.longitude, msg.gps_qual, msg.num_sats, msg.altitude]
                except:
                    pass
                row += gps_data

            sensor_line = sensor_ser.readline().decode("utf-8").strip()
            parts = sensor_line.split('|') if sensor_line else []
            mv, mp, temp = '', '', ''
            if len(parts) == 3:
                try:
                    mv = parts[0].split(':')[1].strip()
                    mp = parts[1].split(':')[1].strip().replace('%', '')
                    temp = parts[2].split(':')[1].strip().replace('°C', '')
                except:
                    pass

            if user_config.get('moisture_value'):
                row.append(mv)
            if user_config.get('moisture_percent'):
                row.append(mp)
            if user_config.get('temperature'):
                row.append(temp)

            writer.writerow(row)
            csvfile.flush()
            time.sleep(1)

        sensor_ser.close()
        if gps_ser:
            gps_ser.close()

@app.route('/', methods=['GET', 'POST'])
def index():
    global is_logging, log_thread, user_config
    if request.method == 'POST':
        if 'start' in request.form:
            user_config = {
                'gps': 'gps' in request.form,
                'moisture_value': 'moisture_value' in request.form,
                'moisture_percent': 'moisture_percent' in request.form,
                'temperature': 'temperature' in request.form
            }
            is_logging = True
            log_thread = threading.Thread(target=read_data)
            log_thread.start()
        elif 'stop' in request.form:
            is_logging = False
            if log_thread:
                log_thread.join()
        return redirect(url_for('index'))
    return render_template('index.html', is_logging=is_logging)

@app.route('/downloads')
def list_downloads():
    files = os.listdir("data")
    return render_template('downloads.html', files=files)

@app.route('/download/<filename>')
def download_file(filename):
    return send_from_directory("data", filename, as_attachment=True)

if __name__ == '__main__':
    os.makedirs('data', exist_ok=True)
    app.run(host='0.0.0.0', port=5000, debug=True)
