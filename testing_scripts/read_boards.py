from scservo_sdk import sms_sts, PortHandler
import time
import helper
import presets

front_port = "/dev/cu.usbmodem5AB90675711"
back_port = "/dev/cu.usbmodem5AB90680351"

front_servo_id = [2, 6, 3, 5]
back_servo_id = [4, 6, 1, 2]

BAUDRATE = 1000000
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42

front_servo_port, front_servo = helper.make_port_handler(front_port)
back_servo_port, back_servo = helper.make_port_handler(back_port)

print("pinging front servo...")
for id in front_servo_id:
    helper.ping_servo(front_servo, id)
print("pinging back servo...")
for id in back_servo_id:
    helper.ping_servo(back_servo, id)

print("front")
for id in front_servo_id:
    helper.read_servo_for_config(front_servo, id)
print("back")
for id in back_servo_id:
    helper.read_servo_for_config(back_servo, id)