import serial
import time

# Replace with your actual port from Step 2
# It is usually '/dev/ttyACM0' or '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

try:
    # Initialize the serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Give the connection a second to settle
    time.sleep(2)
    print("Connection established! Listening for data...\n")
    print("-" * 30)

    while True:
        # Check if there is data waiting in the serial buffer
        if ser.in_waiting > 0:
            # Read the line, decode the bytes into a string, and strip extra whitespace/newlines
            line = ser.readline().decode('utf-8').rstrip()
            print(f"Received: {line}")

except serial.SerialException as e:
    print(f"\nSerial Error: Could not open port {SERIAL_PORT}. Is it plugged in?")
    print(e)
except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    # Always cleanly close the serial port when done
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")