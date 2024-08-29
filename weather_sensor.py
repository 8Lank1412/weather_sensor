import RPi.GPIO as GPIO
import serial
import time
import struct
import paho.mqtt.client as mqtt
import json


# MQTT settings
MQTT_BROKER = "100.103.112.70"
MQTT_PORT = 1883
MQTT_TOPIC = f"weather_sensor/{RASPBERRY_PI_ID}/data"


# Serial port settings
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 9600


# Initialize MQTT client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)


def hex_to_float(hex_value):
    """Convert hexadecimal to float."""
    return struct.unpack('>h', bytes.fromhex(hex_value))[0] * 0.1


def process_data(data):
    """Process the sensor data and return as a dictionary."""
    if len(data) < 21:  # Check if we have enough data
        print("Incomplete data received")
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


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


    print("Press Ctrl + C to exit")
    try:
        while True:
            hexInput = bytes.fromhex("0B0301F4000804A8")
            ser.write(hexInput)
            time.sleep(0.1)
            buf = []
            while ser.inWaiting():
                buf.append(ser.read().hex())
            if buf:
                print("Raw data:", buf)
                processed_data = process_data(buf)
                if processed_data:
                    print("Processed data:", processed_data)
                    # Publish to MQTT
                    client.publish(MQTT_TOPIC, json.dumps(processed_data))
            else:
                print("No data received")
            time.sleep(5)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        ser.flush()
        ser.close()
        client.disconnect()


if __name__ == "__main__":
    main()
