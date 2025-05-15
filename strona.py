#pamietać o zainstalowaniu Flask:
#pip install Flask
#sudo python3 strona.py

#wejscie z innego urzadzenia
#http://<IP_RaspberryPI>


from flask import Flask, render_template, request
from threading import Thread

app = Flask(__name__)
robot = Robot() #musi być gotowa klasa Robot()

@app.route('/')
def index():
  return render_template('index.html')

@app.route('/start', methods=['POST'])
def start():
  x = float(request.form['x'])  #dlugosc sciany
  z = float(request.form['z'])  #wysokosc sciany

  #check pozycji startowej
  if not robot.endstop_floor_1.actual_state:
    print("[INFO] Robot nie jest w pozycji startowej - przestawianie...")
    #wywołanie kodu powrotu do pozycji startowej
    # robot_pos_start
  
  def run_robot():
    print(f"[START] Malowanie: X={x}cm, Z={z}cm")
    robot.move_forward(x)
    #tutaj wywołanie obsługi ruchu góra doł - malowanie
    # robot.move_vertical(z)

  thread = Thread(target=run_robot)
  thread.start()

  return "Robot rozpoczal prace"

if __name__ == '__main__':
  app.run(host=0.0.0.0, port=80)  #dostep LAN z innych urzadzeń
