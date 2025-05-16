from flask import Flask, render_template, request, jsonify
from threading import Thread

# Import klas z Ruch_silnikami.py (odkomentowac, gdy plik będzie gotowy do użycia)
# from Ruch_silnikami import Robot, Motor, Ultrasonic_sensor, Endstop, Ncoder

app = Flask(__name__)
status_log = []

#Funkcja logująca
def log(source, message, type="info"):
    status_log.append({"source": source, "message": message, "type": type})

# Inicjalizacja klas (odkomentowac, gdy gotowe)
# robot = Robot(...)
# motor_left = Motor(...)
# motor_right = Motor(...)
# ultrasonic = Ultrasonic_sensor(...)
# endstop1 = Endstop(...)
# encoder = Ncoder(...)

@app.route('/')
def index():
    return render_template('panel.html')

@app.route('/start', methods=['POST'])
def start():
    status_log.clear()
    try:
        data = request.get_json()
        x = float(data['x'])
        z = float(data['z'])
    except (KeyError, ValueError, TypeError) as e:
        msg = f"Błędne dane wejściowe: {e}"
        log("Panel", msg, "error")
        return jsonify({"status": "error", "message": msg, "logs": status_log}), 400

    def run_robot():
        log("Robot", f"Start ruchu: X={x} cm, Z={z} cm", "info")
        # Przykładowe wywołania metod i logowanie (odkomentowac i dostosowac po integracji)
        # robot.move_forward(x)
        # log("Motor_Left", "Lewy silnik uruchomiony", "info")
        # log("Motor_Right", "Prawy silnik uruchomiony", "info")
        # log("Ultrasonic_sensor", f"Pomiar odległości: ... cm", "info")
        # log("Endstop1", f"Stan krańcówki: ...", "info")
        # log("Ncoder", f"Pozycja enkodera: ...", "info")
        log("Robot", "Ruch zakończony", "success")

    thread = Thread(target=run_robot)
    thread.start()

    log("Panel", "Robot rozpoczął pracę", "success")
    return jsonify({"status": "success", "message": "Robot uruchomiony", "logs": status_log})

@app.route('/logs', methods=['GET'])
def get_logs():
    return jsonify(status_log)

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, debug=True)
