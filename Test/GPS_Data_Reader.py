import serial

ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

try:
    while True:
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        if data:
            print(data) 
except KeyboardInterrupt:
    pass

finally:
    ser.close()
