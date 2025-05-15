class FakeEndstop:
    def __init__(self):
        self.actual_state = True  # Zakładamy, że robot jest w pozycji startowej

class Robot:
    def __init__(self, status_log):
        self.status_log = status_log
        self.is_moving = False
        self.endstop_floor_1 = FakeEndstop()
        self.log("[Robot] - Initialized successfully")

    def log(self, message, type="success"):
        self.status_log.append({"source": "[Robot]", "message": message, "type": type})

    def move_forward(self, distance):
        try:
            self.is_moving = True
            self.log(f"Started moving forward: {distance} cm", "info")
            steps = int(distance // 10)
            for i in range(steps):
                self.log(f"Moving... {i * 10} / {distance} cm", "info")
            self.log(f"Finished moving forward: {distance} cm", "success")
        except Exception as e:
            self.log(f"Error while moving: {e}", "error")
        finally:
            self.is_moving = False

    def stop(self):
        if self.is_moving:
            self.log("Robot stopped", "info")
            self.is_moving = False
        else:
            self.log("Stop command received, but robot was not moving", "info")
