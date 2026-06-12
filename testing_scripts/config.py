from robot.toirtis import Toirtis

import serial.tools.list_ports
import time

# List all available serial ports
ports = serial.tools.list_ports.comports()

print("Available serial ports:")
for port in ports:
    print(f"  {port.device}")
    print(f"    Description: {port.description}")
    print(f"    Manufacturer: {port.manufacturer}")
    print()


from scservo_sdk import sms_sts, PortHandler

# Configuration
BAUDRATE = 1000000
SERVO_ID = 1

# Initialize the port handler and servo handler
port = '/dev/cu.usbmodem5AB90680351'
port_handler = PortHandler(port)
servo = sms_sts(port_handler)

# Open the serial port
if port_handler.openPort():
    print("✓ Port opened successfully")
else:
    print("✗ Failed to open port")
    
# Set the baud rate
if port_handler.setBaudRate(BAUDRATE):
    print(f"✓ Baud rate set to {BAUDRATE}")
else:
    print("✗ Failed to set baud rate")

found = []

# Scan IDs 0–253
for servo_id in range(0, 10):
    model_number, result, error = servo.ping(servo_id)

    if result == 0:
        print(f"Found servo ID: {servo_id}  Model: {model_number}")
        found.append(servo_id)

print("\nDone")
print("Servos found:", found)


# Ping the servo to check communication
model_number, comm_result, error = servo.ping(SERVO_ID)

if comm_result == 0:  # COMM_SUCCESS
    print(f"✓ Servo ID {SERVO_ID} found!")
    print(f"  Model number: {model_number}")
else:
    print(f"✗ Failed to ping servo")

# Register addresses
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
ADDR_TORQUE_ENABLE = 40

def read_servo_data(servo_id):
    print(f"\nReading data from Servo ID {servo_id}:")
    position, comm_result, error = servo.read2ByteTxRx(servo_id, ADDR_PRESENT_POSITION)
    if comm_result == 0:
        degrees = position * 360 / 4096  # Convert to degrees
        print(f"Current Position: {position} (raw) = {degrees:.1f}°")
    else:
        print(f"Failed to read position")

    voltage_raw, comm_result, error = servo.read1ByteTxRx(servo_id, ADDR_PRESENT_VOLTAGE)
    if comm_result == 0:
        voltage = voltage_raw * 0.1  # Convert to volts
        print(f"Voltage: {voltage:.1f}V")
    else:
        print(f"Failed to read voltage")

    temp, comm_result, error = servo.read1ByteTxRx(servo_id, ADDR_PRESENT_TEMPERATURE)
    if comm_result == 0:
        print(f"Temperature: {temp}°C")
    else:
        print(f"Failed to read temperature")

    torque_enabled, comm_result, error = servo.read1ByteTxRx(servo_id, ADDR_TORQUE_ENABLE)
    if comm_result == 0:
        print(f"Torque Enabled: {'Yes' if torque_enabled else 'No'}")
    else:
        print(f"Failed to read torque status")

