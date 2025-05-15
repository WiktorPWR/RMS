#-----------------------------------------------------------------------------
#jak to ma wygladac w glownym kodzie:
#w każdej strukturze mają znaleźć się elementy z poniższych komentarzy  
class Robot:
    def __init__(self):
        self.status_log = []    #dodanie log w init klasy
        
    #funkcja komunikatów:
    def log(self, msg):
        print(f"[Robot] {msg}")    #tutaj [......] dla kazdej klasy jej nazwa
        self.status_log.append(msg)

    #tak ma wygladac wstawienie komunikatów:
    def move_forward(self, distance):
        self.log(f"Jade do przodu o {distance} cm")

    def move_vertical(self, height):
        self.log(f"Maluje pionowo na wysokosc {height} cm")
#-----------------------------------------------------------------------------
#jak to ma wygladac w pliku strona.py:
@app.route('/start', methods=['POST'])
def start():
    x = float(request.form['x'])
    z = float(request.form['z'])

    if not robot.endstop_floor_1.actual_state:
        flash("Robot nie był w pozycji startowej – przestawianie...", "warning")

    def run_robot():
        msg, category = robot.move_forward(x)
        flash(msg, category)

    Thread(target=run_robot).start()
    flash("Robot rozpoczął pracę", "success")
    return redirect(url_for('index'))
