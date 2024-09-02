# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import serial
import time
import struct

# if use half-auto, EN_485 = LOW is Receiver, EN_485 = HIGH is Send
MODE = 0  # mode = 0 is full-auto, mode = 1 is half-auto
if MODE == 1:
    EN_485 = 4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EN_485, GPIO.OUT)
    GPIO.output(EN_485, GPIO.HIGH)

ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)

def hex_to_float(hex_value):
    """Convert hexadecimal to float."""
    return struct.unpack('>h', bytes.fromhex(hex_value))[0] * 0.1

def process_data(data):
    """Process and print the sensor data."""
    if len(data) < 21:  # Check if we have enough data
        print("Incomplete data received")
        return

    humidity = hex_to_float(data[3] + data[4])
    temperature = hex_to_float(data[5] + data[6])
    noise = hex_to_float(data[7] + data[8])
    pm25 = int(data[9] + data[10], 16)
    pm10 = int(data[11] + data[12], 16)
    pressure = hex_to_float(data[13] + data[14])
    lux = int(data[15] + data[16] + data[17] + data[18], 16)

    print(f"Humidity: {humidity:.1f}%")
    print(f"Temperature: {temperature:.1f}°C")
    print(f"Noise Level: {noise:.1f} dB")
    print(f"PM2.5: {pm25} μg/m3")
    print(f"PM10: {pm10} μg/m3")
    print(f"Atmospheric Pressure: {pressure:.1f} kPa")
    print(f"Light: {lux} Lux")

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
            process_data(buf)
        else:
            print("No data received")
        time.sleep(5)
except KeyboardInterrupt:
    print("Program terminated by user")
finally:
    ser.flush()
    ser.close()
    if MODE == 1:
        GPIO.cleanup()
