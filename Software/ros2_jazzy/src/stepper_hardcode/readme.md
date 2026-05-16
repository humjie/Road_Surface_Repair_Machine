python3 -c "import serial; s = serial.Serial('/dev/esp_stepper', 115200); s.write(b'S'); print('sent')"

ros2 run scan_trigger scan_trigger_node