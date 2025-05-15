#pamietać o zainstalowaniu Flask:
#pip install Flask
#sudo python3 strona.py

#wejscie z innego urzadzenia
#http://<IP_RaspberryPI>


from flask import Flask, render_template, request, jsonify
from threading import Thread
from test_robot import Robot


app = Flask(__name__)
status_log = []
robot = Robot(status_log) #musi być gotowa klasa Robot()

@app.route('/')
def index():
  return render_template('strona.html')

@app.route('/start', methods=['POST'])
def start():
  data = request.get_json()
  status_log.clear()
  try:
    x = float(data['x'])  #dlugosc sciany
    z = float(data['z'])  #wysokosc sciany
  except (KeyError, ValueError, TypeError):
    status_log.append({"source": "App", "message": "Błędne dane wejściowe", "type": "error"})
    return jsonify(status_log)

  #check pozycji startowej
  if not robot.endstop_floor_1.actual_state:
    print("[INFO] Robot nie jest w pozycji startowej - przestawianie...")
    #wywołanie kodu powrotu do pozycji startowej
    # robot_pos_start
  
  def run_robot():
    robot.log(f"Malowanie rozpoczęte: X={x} cm, Z={z} cm", "info")
    robot.move_forward(x)
    #tutaj wywołanie obsługi ruchu góra doł - malowanie
    # robot.move_vertical(z)

  thread = Thread(target=run_robot)
  thread.start()

  robot.log("Robot rozpoczął pracę", "success")
  return jsonify(status_log)

@app.route('/logs', methods=['GET'])
def get_logs():
  return jsonify(status_log)

if __name__ == '__main__':
  app.run(host=0.0.0.0, port=80)  #dostep LAN z innych urzadzeń
