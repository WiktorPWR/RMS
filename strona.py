from flask import Flask, render_template, request, jsonify
from threading import Thread
from RMS.test_robot import Robot

app = Flask(__name__)
status_log = []
robot = Robot(status_log)

@app.route('/')
def index():
    return render_template('strona.html')

@app.route('/start', methods=['POST'])
def start():
    status_log.clear()

    try:
        data = request.get_json()
        x = float(data['x'])
        z = float(data['z'])
    except (KeyError, ValueError, TypeError) as e:
        msg = f"Błędne dane wejściowe: {e}"
        print("[BACKEND]", msg)
        robot.log(msg, "error")
        return jsonify({"status": "error", "message": msg, "logs": status_log}), 400

    if hasattr(robot, "endstop_floor_1") and not robot.endstop_floor_1.actual_state:
        robot.log("Robot nie jest w pozycji startowej - przestawianie...", "info")

    def run_robot():
        robot.log(f"Malowanie rozpoczęte: X={x} cm, Z={z} cm", "info")
        robot.move_forward(x)
        robot.log("Zakończono malowanie", "success")

    thread = Thread(target=run_robot)
    thread.start()

    robot.log("Robot rozpoczął pracę", "success")
    return jsonify({"status": "success", "message": "Robot uruchomiony", "logs": status_log})

@app.route('/logs', methods=['GET'])
def get_logs():
    return jsonify(status_log)

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, debug=True)
