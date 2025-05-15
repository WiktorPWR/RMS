#plik do testowania strony na sucho bez połaczenia z Raspberry
class Robot:
    """
    Mock class for testing wall-painting robot.
    Simulates robot behavior and logs all messages to status_log.
    """

    def __init__(self, status_log):
        """
        Initializes the Robot with a shared status_log list.

        Parameters
        ----------
        status_log : list
            List used to collect status messages (successes, errors, etc.).
        """
        self.status_log = status_log
        self.is_moving = False
        self.log("[Robot] - Initialized successfully")

    def log(self, message, type="success"):
        """
        Logs a message to the status_log list.

        Parameters
        ----------
        message : str
            Message to log.
        type : str
            Type of message: 'success', 'error', 'info'.
        """
        self.status_log.append({"source": "Robot", "message": message, "type": type})

    def move_forward(self, distance):
        """
        Simulates movement forward.

        Parameters
        ----------
        distance : float
            Distance to move forward (in cm).
        """
        try:
            self.is_moving = True
            self.log(f"Started moving forward: {distance} cm", "info")

            # Simulate step-by-step progress
            steps = int(distance // 10)
            for i in range(steps):
                self.log(f"Moving... {i * 10} / {distance} cm", "info")

            self.log(f"Finished moving forward: {distance} cm", "success")
        except Exception as e:
            self.log(f"Error while moving: {e}", "error")
        finally:
            self.is_moving = False

    def stop(self):
        """
        Simulates stopping the robot.
        """
        if self.is_moving:
            self.log("Robot stopped", "info")
            self.is_moving = False
        else:
            self.log("Stop command received, but robot was not moving", "info")
