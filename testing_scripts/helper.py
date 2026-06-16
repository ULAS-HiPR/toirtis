from scservo_sdk import sms_sts, PortHandler

BAUDRATE = 1000000
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42

def make_port_handler(port):
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

    return port_handler, servo

def ping_servo(servo, servo_id):
    # Ping the servo to check if it's responsive
    model_number, result, error = servo.ping(servo_id)

    if result == 0:
        print(f"Found servo ID: {servo_id}  Model: {model_number}")
    else:
        print(f"✗ Failed to ping servo ID {servo_id}")

def move_servo(servo, servo_id, position, speed=3400, acceleration=50):
    servo.WritePosEx(servo_id, position,  speed, acceleration)
    degrees = position * 360 / 4096  # Convert to degrees
    print(f"{servo_id} Moving to position ({position} = {degrees}°)")


def read_servo_data(servo, servo_id):
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

def read_servo_for_config(servo, servo_id):
    position, comm_result, error = servo.read2ByteTxRx(servo_id, ADDR_PRESENT_POSITION)
    if comm_result == 0:
        degrees = position * 360 / 4096  # Convert to degrees
        print(f"({servo_id} , {position}), # {degrees:.1f}°")
    else:
        print(f"Failed to read position")

def enable_torque(servo, servo_id, enable=True):
    servo.write1ByteTxRx(servo_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)
    print(f"{'Enabled' if enable else 'Disabled'} torque for Servo ID {servo_id}")

def disable_torque(servo, servo_id):
    enable_torque(servo, servo_id, enable=False)

def set_limits(servo, servo_id, angles=(0, 4095)):
    min_angle, max_angle = angles
    servo.write2ByteTxRx(servo_id, 9, min_angle)  # Min angle limit
    servo.write2ByteTxRx(servo_id, 11, max_angle)  # Max angle limit
    print(f"Set angle limits for Servo ID {servo_id}: Min={min_angle}, Max={max_angle}")