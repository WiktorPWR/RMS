from flask import Flask, render_template, request, jsonify
from datetime import datetime
from threading import Thread

# Import klas z Ruch_silnikami.py (odkomentować, gdy plik będzie gotowy do użycia)
# from Ruch_silnikami import Robot, Motor, Ultrasonic_sensor, Endstop, Ncoder

app = Flask(__name__)

# Globalne logi i historia (zerowane po restarcie serwera)
status_log = []
history_log = []

# Funkcja logująca
def log(source, message, type="info"):
    status_log.append({"source": source, "message": message, "type": type})

# Inicjalizacja klas (odkomentować, gdy gotowe)
# robot = Robot(...)
# motor_left = Motor(...)
# motor_right = Motor(...)
# ultrasonic = Ultrasonic_sensor(...)
# endstop1 = Endstop(...)
# encoder = Ncoder(...)

@app.route('/')
def index():
    """Strona główna z panelem sterowania."""
    return render_template('panel.html')

@app.route('/start', methods=['POST'])
def start():
    """Uruchomienie malowania i zapis do historii."""
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
        log("Robot", f"Malowanie rozpoczęte: X={x} cm, Z={z} cm", "info")
        # Przykładowe wywołania metod i logowanie (odkomentować i dostosować po integracji)
        # robot.move_forward(x)
        # paint_module.spray(2)
        log("Robot", "Zakończono malowanie", "success")
        # Dodaj wpis do historii
        history_log.append({
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "x": x,
            "z": z,
            "status": "Zakończono malowanie"
        })

    thread = Thread(target=run_robot)
    thread.start()

    log("Panel", "Robot rozpoczął pracę", "success")
    return jsonify({"status": "success", "message": "Robot uruchomiony", "logs": status_log})

@app.route('/calibrate', methods=['POST'])
def calibrate():
    """Uruchomienie kalibracji i zapis do historii."""
    status_log.clear()
    def run_calibration():
        # kalibracja.calibrate()  # Odkomentować po integracji
        log("Robot", "Kalibracja zakończona", "success")
        history_log.append({
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "x": "-",
            "z": "-",
            "status": "Kalibracja"
        })
    thread = Thread(target=run_calibration)
    thread.start()
    log("Panel", "Rozpoczęto kalibrację", "info")
    return jsonify({"status": "success", "message": "Kalibracja uruchomiona", "logs": status_log})

@app.route('/logs', methods=['GET'])
def get_logs():
    """Zwraca aktualne logi."""
    return jsonify(status_log)

@app.route('/reset_logs', methods=['POST'])
def reset_logs():
    """Czyści logi."""
    status_log.clear()
    return jsonify({"status": "success"})

@app.route('/history', methods=['GET'])
def get_history():
    """Zwraca historię wykonań."""
    return jsonify(history_log)

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5050, debug=True)
