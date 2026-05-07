import serial
import time

class RobotArm:
    SERVO_NAMES = ["base", "shoulder", "elbow", "wrist", "claw", "gripper"]

    TICK_MIN  = [120, 150, 120, 140, 120, 230]
    TICK_REST = [325, 470, 400, 519, 200, 300] # purely cosmetic
    TICK_MAX  = [530, 530, 409, 520, 480, 370]

    BASE     = 0
    SHOULDER = 1
    ELBOW    = 2
    WRIST    = 3
    CLAW     = 4
    GRIPPER  = 5

    def __init__(self, port, baud=115200, timeout=5.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def connect(self):
        print(f"Connecting to {self.port} at {self.baud} baud...")
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        
        deadline = time.time() + 10
        while time.time() < deadline:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if line == "At rest":
                print("Arduino ready.")
                return
            elif line:
                print(f"  (startup) {line}")
        
        raise TimeoutError("Arduino did not respond within 10 seconds.")
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected.")
        else:
            print("Already disconnected.")
    
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
    
    # internal method only due to starting underscore, can only be used within other methods
    def _send(self, message):
        self.ser.write((message.strip() + "\n").encode("ascii"))
        response = self.ser.readline().decode("ascii", errors="ignore").strip()
        if response == "ERR":
            raise ValueError(f"Arduino rejected command: {message!r}")
        return response
    
    def _clamp(self, servo_id, tick):
        clamped = max(self.TICK_MIN[servo_id], min(self.TICK_MAX[servo_id], int(tick)))
        if clamped != int(tick):
            print(f"Warning: {self.SERVO_NAMES[servo_id]} tick {tick} clamped to {clamped} (range {self.TICK_MIN[servo_id]}–{self.TICK_MAX[servo_id]})")
        return clamped

    # S for moving one servo, M for moving multiple servos at once, R for rest, and Q for querying current positions
    def move(self, servo_id, tick):
        tick = self._clamp(servo_id, tick)
        self._send(f"S{servo_id},{tick}")

    def move_multi(self, positions):
        pairs = []
        for servo_id, tick in positions.items():
            tick = self._clamp(servo_id, tick)
            pairs.append(f"{servo_id},{tick}")
        self._send("M" + ",".join(pairs))

    # after piece has been picked up
    def move_multi_preset(self):
        self.move_multi({
            self.SHOULDER: 450,
            self.ELBOW: 300,
            self.CLAW: 200
    })

    def rest(self):
        print("Moving to rest...")
        self._send("R")

    def query(self):
        response = self._send("Q")
        result = {}  # dictionary
        for part in response.split(","):
            name, tick = part.split(":")
            result[name] = int(tick)
        return result

    def open_gripper(self):
        self.move(self.GRIPPER, self.TICK_MAX[self.GRIPPER])

    def close_gripper(self):
        self.move(self.GRIPPER, self.TICK_MIN[self.GRIPPER])

# if file is run in terminal, code will execute
if __name__ == "__main__":
    PORT = "COM6"

    with RobotArm(PORT) as arm:
        print("Current state:", arm.query())
        arm.rest()