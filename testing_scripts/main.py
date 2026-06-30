from scservo_sdk import sms_sts, PortHandler
import time
import helper
import presets

front_port = "/dev/ttyACM1"
back_port = "/dev/ttyACM0"

front_servo_id = [2, 6, 3, 5]
back_servo_id = [4, 6, 1, 2]

BAUDRATE = 1000000
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42

def do_can_to_stand(front_servo, back_servo):
    print(f"moving to legs")
    for id, pos in presets.front_legs_pos:
        helper.move_servo(front_servo, id, pos)
    for id, pos in presets.back_legs_pos:
        helper.move_servo(back_servo, id, pos)

    time.sleep(5)

    print("\nMoving to can")
    for id, pos in presets.front_can_pos:
        helper.move_servo(front_servo, id, pos)
    for id, pos in presets.back_can_pos:
        helper.move_servo(back_servo, id, pos)

    time.sleep(5)

    for id in front_servo_id:
        helper.read_servo_data(front_servo, id)
    for id in back_servo_id:
        helper.read_servo_data(back_servo, id)


    #print(f"moving to legs")
    #for id, pos in presets.front_legs_pos:
    #    helper.move_servo(front_servo, id, pos)
    #for id, pos in presets.back_legs_pos:
    #    helper.move_servo(back_servo, id, pos)
#
    #time.sleep(5)
#
#
    #print(f"moving to stand")
    #for id, pos in presets.front_stand_pos:
    #    helper.move_servo(front_servo, id, pos)
    #for id, pos in presets.back_stand_pos:
    #    helper.move_servo(back_servo, id, pos)
    #
    #time.sleep(5)

def do_wave(front_servo, back_servo):
    print(f"waving")
    for id, pos in presets.back_wave_pos:
        helper.move_servo(back_servo, id, pos)
    for id, pos in presets.front_wave_down_pos:
        helper.move_servo(front_servo, id, pos)
    time.sleep(2)

    for id, pos in presets.front_wave_down_pos:
        helper.move_servo(front_servo, id, pos, 3400, 25)
    time.sleep(1)
    for id, pos in presets.front_wave_up_pos:
        helper.move_servo(front_servo, id, pos, 3400, 25)
    time.sleep(1)
    for id, pos in presets.front_wave_down_pos:
        helper.move_servo(front_servo, id, pos, 3400, 25)
    time.sleep(1.5)
    for id, pos in presets.front_wave_up_pos:
        helper.move_servo(front_servo, id, pos, 3400, 25)
    time.sleep(1.5)


    for id, pos in presets.front_wave_down_pos:
        helper.move_servo(front_servo, id, pos)
    for id, pos in presets.back_stand_pos:
        helper.move_servo(back_servo, id, pos)
    time.sleep(5)

def do_walk(front_servo, back_servo, steps=4):
    for step in range(steps):
        print(f"Step {step + 1} of {steps}")
        for front_pos, back_pos in presets.walking_cycle:
            for id, pos in front_pos:
                helper.move_servo(front_servo, id, pos)
            for id, pos in back_pos:
                helper.move_servo(back_servo, id, pos)
            time.sleep(0.4)  # Adjust timing as needed

def do_rock(front_servo, back_servo):
    for front_pos, back_pos in presets.rocking_cycle:
        for id, pos in front_pos:
            helper.move_servo(front_servo, id, pos, 3400, 100)
        for id, pos in back_pos:
            helper.move_servo(back_servo, id, pos, 3400, 100)
        time.sleep(1.5)  # Adjust timing as needed

def main():
    front_servo_port, front_servo = helper.make_port_handler(front_port)
    back_servo_port, back_servo = helper.make_port_handler(back_port)

    print("pinging front servo...")
    for id in front_servo_id:
        helper.ping_servo(front_servo, id)
    print("pinging back servo...")
    for id in back_servo_id:
        helper.ping_servo(back_servo, id)

    for id in front_servo_id:
        helper.set_limits(front_servo, id, presets.front_servo_min_max[id])
    for id in back_servo_id:
        helper.set_limits(back_servo, id, presets.back_servo_min_max[id])

    for id in front_servo_id:
        helper.enable_torque(front_servo, id)
    for id in back_servo_id:
        helper.enable_torque(back_servo, id)

    for id in front_servo_id:
        helper.read_servo_data(front_servo, id)
    for id in back_servo_id:
        helper.read_servo_data(back_servo, id)

    #do_can_to_stand(front_servo, back_servo)
    do_walk(front_servo, back_servo, steps=4) #about a full cicle
    #do_wave(front_servo, back_servo)

    #do_rock(front_servo, back_servo)

    for id in front_servo_id:
        helper.read_servo_data(front_servo, id)
    for id in back_servo_id:
        helper.read_servo_data(back_servo, id)

    for id in front_servo_id:
        helper.disable_torque(front_servo, id)
    for id in back_servo_id:
        helper.disable_torque(back_servo, id)

if __name__ == "__main__":
    main()