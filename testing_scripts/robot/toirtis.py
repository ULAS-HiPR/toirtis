from .leg import Leg

class Toirtis:
    def __init__(self, front_controller, back_controller):
        self.front_left_leg = Leg("front_left")
        self.front_right_leg = Leg("front_right")
        self.back_left_leg = Leg("back_left")
        self.back_right_leg = Leg("back_right")
        self.front_controller = front_controller
        self.back_controller = back_controller

    def move(self, direction):
        print(f"Moving {direction}")

    def stop(self):
        print("Stopping")