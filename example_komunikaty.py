#-----------------------------------------------------------------------------
#jak to ma wygladac w glownym kodzie:
#w każdej strukturze mają znaleźć się elementy z poniższych komentarzy  
class Robot:
    def __init__(self, status_log):
        self.status_log = status_log    #do init dodajecie ten wiersz
        self.is_moving = False
        self.endstop_floor_1 = FakeEndstop()
        self.log("[Robot] - Initialized successfully") 
    #dajecie funkcje log (po prostu Ctrl C, Ctrl V - tylko zmieniacie nazwe z "Robot" na swoją np "Servo")
    def log(self, message, type="success"):
        self.status_log.append({"source": "[Robot]", "message": message, "type": type})

    def move_forward(self, distance):
        try:
            self.is_moving = True
            #-------------------------------------------------------------------------------------
            #tutaj przykład wyświetlania tekstu
            #ogólnie dajecie self.log zamiast print (f"wasz tekst komunikatu", "typ komunikatu")
            #typy jakie macie do wyboru to: info, warning, error, success
            self.log(f"Started moving forward: {distance} cm", "info")    
            #-------------------------------------------------------------------------------------
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

#-----------------------------------------------------------------------------
#TO MACIE W DUPIE, ROBICIE TYLKO TO CO WYŻEJ
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



#----------------------------------------------------------------------
#main

class MAIN():
    
    def praca_robot(self, x,z)
    
    def move_z_axis(self, distance, paint_or_not=False):
    i = [0, 10, 20]
    for.......
    
        def move_z_axis(self, -i, paint_or_not=False):
            delay
        def press(self):
        def move_forward(self, (parzyste nieparzyste +/-)distance):
        def release(self):
        

#--------------------------------------------------------------------
#panel.py

    def run_robot():
        log("Robot", f"Malowanie rozpoczęte: X={x} cm, Z={z} cm", "info")
        # Przykładowe wywołania metod i logowanie (odkomentować i dostosować po integracji)
        # main.praca_robot(x,z)
        log("Robot", "Zakończono malowanie", "success")
        # Dodaj wpis do historii
        history_log.append({
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "x": x,
            "z": z,
            "status": "Zakończono malowanie"
        })





