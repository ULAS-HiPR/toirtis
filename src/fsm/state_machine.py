from enum import IntEnum
import math


class State(IntEnum):
    CALIBRATING = 0
    READY = 1
    POWERED = 2
    COASTING = 3
    DROUGE = 4
    MAIN = 5
    LANDED = 6


class FlightStateMachine:
    def __init__(self, main_height=200, liftoff_threshold=20, drouge_delay=0):
        self.main_height = main_height
        self.liftoff_threshold = liftoff_threshold
        self.drouge_delay = drouge_delay

        self.state = State.CALIBRATING
        self.called_once = {s: False for s in State}

        self.change_state(State.READY)

    def change_state(self, new_state: State):
        if new_state != self.state:
            if not self.called_once[new_state]:
                self.on_enter(new_state)
                self.called_once[new_state] = True
            self.state = new_state

    def on_enter(self, state: State):
        print(f"[STATE ENTER] {state.name}")

    def update(self, pred):
        accel = pred["acceleration"]
        vel = pred["velocity"]
        alt = pred["altitude"]

        if self.state == State.CALIBRATING:
            self.change_state(State.READY)

        elif self.state == State.READY:
            if abs(accel) > self.liftoff_threshold:
                print(f"Liftoff detected: accel={accel:.2f} m/s², vel={vel:.2f} m/s, alt={alt:.2f} m")
                self.change_state(State.POWERED)

        elif self.state == State.POWERED:
            if accel < 0:
                self.change_state(State.COASTING)

        elif self.state == State.COASTING:
            if vel < 0:
                self.change_state(State.DROUGE)

        elif self.state == State.DROUGE:
            if alt < self.main_height:
                self.change_state(State.MAIN)

        elif self.state == State.MAIN:
            if vel < 0 or accel < 0.2:
                self.change_state(State.LANDED)

        elif self.state == State.LANDED:
            print("LANDED")