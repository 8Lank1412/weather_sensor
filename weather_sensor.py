import RPi.GPIO as GPIO
import serial
import time
import struct
import paho.mqtt.client as mqtt
import json
import socket

hostname = socket.gethostname()
ip = socket.gethostbyname(hostname)
ID = hostname + ip

# MQTT settings
MQTT_BROKER = "100.103.112.70"
MQTT_PORT = 1883
MQTT_TOPIC = f"weather_sensor/{ID}/data"

# Serial port settings
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 9600

# Initialize MQTT client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

def hex_to_float(hex_value):
    """Convert hexadecimal to float."""
    return struct.unpack('>h', bytes.fromhex(hex_value))[0] * 0.1

def process_weather_data(data):
    """Process the weather sensor data and return as a dictionary."""
    if len(data) < 21:  # Check if we have enough data
        print("Incomplete weather data received")
        return None
    return {
        "humidity": hex_to_float(data[3] + data[4]),
        "temperature": hex_to_float(data[5] + data[6]),
        "noise": hex_to_float(data[7] + data[8]),
        "pm25": int(data[9] + data[10], 16),
        "pm10": int(data[11] + data[12], 16),
        "pressure": hex_to_float(data[13] + data[14]),
        "lux": int(data[15] + data[16] + data[17] + data[18], 16)
    }

def read_wind_direction(ser):
    """Read wind direction data from the sensor."""
    query = bytes.fromhex("0D0300000002C4C7")
    ser.write(query)
    time.sleep(0.1)
    response = ser.read(9)
    if len(response) == 9 and response[0] == 0x0D and response[1] == 0x03:
        direction_decimal = struct.unpack('>H', response[3:5])[0] * 0.1
        direction = struct.unpack('>H', response[5:7])[0]
        return {"wind_direction_decimal": direction_decimal, "wind_direction": direction}
    return None

def read_wind_speed(ser):
    """Read wind speed data from the sensor."""
    query = bytes.fromhex("0C03000000018517")
    ser.write(query)
    time.sleep(0.1)
    response = ser.read(7)
    if len(response) == 7 and response[0] == 0x0C and response[1] == 0x03:
        speed = struct.unpack('>H', response[3:5])[0] * 0.1
        return {"wind_speed": speed}
    return None

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Press Ctrl + C to exit")
    try:
        while True:
            # Read weather sensor data
            hexInput = bytes.fromhex("0B0301F4000804A8")
            ser.write(hexInput)
            time.sleep(0.1)
            buf = []
            while ser.inWaiting():
                buf.append(ser.read().hex())
            
            all_data = {}
            if buf:
                print("Raw weather data:", buf)
                weather_data = process_weather_data(buf)
                if weather_data:
                    all_data.update(weather_data)
            else:
                print("No weather data received")

            # Read wind direction data
            wind_direction = read_wind_direction(ser)
            if wind_direction:
                all_data.update(wind_direction)
            else:
                print("No wind direction data received")

            # Read wind speed data
            wind_speed = read_wind_speed(ser)
            if wind_speed:
                all_data.update(wind_speed)
            else:
                print("No wind speed data received")

            if all_data:
                print("All processed data:", all_data)
                # Publish to MQTT
                client.publish(MQTT_TOPIC, json.dumps(all_data))
            else:
                print("No data to publish")

            time.sleep(5)

    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        ser.flush()
        ser.close()
        client.disconnect()

if __name__ == "__main__":
    main()
