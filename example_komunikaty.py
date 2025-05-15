#-----------------------------------------------------------------------------
#jak to ma wygladac w glownym kodzie:
class Robot:
    def move_forward(self, x):
        # jakiś test
        if x > 100:
            return ("Błąd: za daleko!", "error")
        print(f"Robot jedzie do przodu o {x} cm")
        return ("Robot jedzie!", "success")
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
