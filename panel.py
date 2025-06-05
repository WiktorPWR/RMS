from flask import Flask, render_template, request, jsonify
from datetime import datetime
from threading import Thread
from main import main_controller, status_log

app = Flask(__name__)

# ------------------------------Globalne logi i historia (zerowane po restarcie serwera)--------------------
history_log = []

def log(source, message, type="info"):
    status_log.append({"source": source, "message": message, "type": type, "time": datetime.now().isoformat()})

#-----------------------------------------------STRONA INTERNETOWA------------------------------------------
@app.route('/')
def index():
    """Strona główna z panelem sterowania."""
    return render_template('panel.html')

#-------------------------------------------------START ROBOTA----------------------------------------------
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
        
        main_controller.malowanie(x, z)

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

#---------------------------------------KALIBRACJA - sprawdzenie komponentów---------------------------------
@app.route('/calibrate', methods=['POST'])
def calibrate():

    status_log.clear()

    def run_calibration():  

        main_controller.kalibracja()

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

#--------------------------------------------LOGI - wyświetlanie---------------------------------------------
@app.route('/logs', methods=['GET'])
def get_logs():

    return jsonify(status_log)

#---------------------------------------------LOGI - czyszczenie---------------------------------------------
@app.route('/reset_logs', methods=['POST'])
def reset_logs():

    status_log.clear()
    return jsonify({"status": "success"})

#------------------------------------------HISTORIA - wyświetlanie-------------------------------------------
@app.route('/history', methods=['GET'])
def get_history():

    return jsonify(history_log)

#----------------------------------------------------IP------------------------------------------------------
if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5050, debug=False)
