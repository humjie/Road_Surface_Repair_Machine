#!/usr/bin/env python3
"""
Simple test script to verify ESP32 stepper communication via /dev/esp_stepper
Run: python3 test_stepper.py
"""

import serial
import time

PORT = "/dev/esp_stepper"
BAUD = 115200  # Change this to match your ESP32 firmware baud rate

def test_stepper():
    print(f"Connecting to {PORT} at {BAUD} baud...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(2)  # Wait for ESP32 to boot/reset after serial open
        print(f"Connected! Port open: {ser.is_open}\n")
    except Exception as e:
        print(f"ERROR: Could not open port: {e}")
        print("Check: Is /dev/esp_stepper listed? Run: ls -l /dev/esp_*")
        return

    # --- Test 1: Send a simple command ---
    print("=== Test 1: Sending command ===")
    command = "MOVE 100\n"   # <-- Change this to match your firmware's expected command
    print(f"Sending: {command.strip()}")
    ser.write(command.encode())
    time.sleep(0.5)

    # Read response
    response = ser.read_all().decode(errors="ignore").strip()
    if response:
        print(f"Response: {response}")
    else:
        print("No response received (check baud rate or firmware command format)")

    # --- Test 2: Read any incoming data for 3 seconds ---
    print("\n=== Test 2: Listening for 3 seconds ===")
    ser.reset_input_buffer()
    start = time.time()
    while time.time() - start < 3:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"Received: {line}")
    print("Done listening.\n")

    # --- Test 3: Check if port is readable ---
    print("=== Test 3: Port info ===")
    print(f"  Port     : {ser.port}")
    print(f"  Baudrate : {ser.baudrate}")
    print(f"  Timeout  : {ser.timeout}")
    print(f"  Open     : {ser.is_open}")

    ser.close()
    print("\nPort closed. Test complete.")

if __name__ == "__main__":
    test_stepper()