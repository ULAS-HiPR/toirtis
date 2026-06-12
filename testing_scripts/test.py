from robot.toirtis import Toirtis

import serial.tools.list_ports
import time

can_pos = {
    2 : 1288,
    3 : 1036,
    5 : 3915,
    6 : 3660
}

legs_pos = {
    2 : 198,
    3 : 1036,
    5 : 3915,
    6 : 2750
}

stand_pos = {
    2 : 3352,
    3 : 327,
    5 : 1000,
    6 : 1676
}

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
SERVO_ID = 6

# Initialize the port handler and servo handler
port_handler = PortHandler("/dev/cu.usbmodem5AB90675711")
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
        print(f"Raw Position: {position}")
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

    moving, comm_result, error = servo.read1ByteTxRx(servo_id, ADDR_MOVING)
    if comm_result == 0:
        print(f"Moving: {'Yes' if moving else 'No'}")
    else:
        print(f"Failed to read moving status")
# Read current position (2 bytes)
read_servo_data(6)

# Register addresses
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
# Disable torque to change EEPROM settings
servo.write1ByteTxRx(SERVO_ID, ADDR_TORQUE_ENABLE, 0)
print("✓ Torque disabled")

servo_min_max = {
    2 : (22,2394),
    3 : (32,1038),
    5 : (1826,2941),
    6 : (482,2890)
}
#
#for SERVO_ID in found:
#    MIN_POS, MAX_POS = servo_min_max[SERVO_ID]
#    servo.write2ByteTxRx(SERVO_ID, ADDR_MIN_ANGLE_LIMIT, MIN_POS)
#    servo.write2ByteTxRx(SERVO_ID, ADDR_MAX_ANGLE_LIMIT, MAX_POS)


# Set mode back to 0 (Position servo mode)
servo.write1ByteTxRx(SERVO_ID, ADDR_MODE, 0)
print("✓ Mode set to 0 (position control)")

# Enable torque
servo.write1ByteTxRx(SERVO_ID, ADDR_TORQUE_ENABLE, 1)
print("✓ Torque enabled")

# Register addresses
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42

# Enable torque
for SERVO_ID in found:
    servo.write1ByteTxRx(SERVO_ID, ADDR_TORQUE_ENABLE, 1)
print("✓ Torque enabled - servo is now holding position")



servo.WritePosEx(6, 3000, 1000, 20)

time.sleep(2)  # Stagger commands slightly

for SERVO_ID in found:
    servo.write1ByteTxRx(SERVO_ID, ADDR_TORQUE_ENABLE, 0)
# Move to position (2048 = 180°)

#move_to = 2048
#degrees = move_to * 360 / 4096  # Convert to degrees
## WritePosEx(ID, Position, Speed, Acceleration)
#servo.WritePosEx(SERVO_ID, move_to, 1000, 50)
#print(f"✓ Moving to position ({move_to} = {degrees}°)")
time.sleep(2)
read_servo_data(6)

port_handler.closePort()

