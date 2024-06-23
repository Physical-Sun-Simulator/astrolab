# Libraries
from flask import Flask, request, render_template, redirect, flash
from astrolab.user_interface_node import user_interface_node
from sun_position import get_sun_elevation_angle, get_sun_azimuth_angle, get_hour_angle, get_true_solar_time, get_equation_of_time, get_sun_declination
import rclpy, math, threading, datetime, sys

# Initialize Flask
app = Flask(__name__)

# Initialize rclpy
rclpy.init(args=None)

# Global variables
node = user_interface_node()

# Constants
PRODUCT_NAME = "αstrolaβ"
OPTIONS = ['simulation', 'calibration', 'dynamics']

############
# Webpages #
############

@app.route("/")
@app.route('/simulation')
def simulation():
    return render_template('simulation.html',
                           title="Simulation",
                           name=PRODUCT_NAME,
                           options=OPTIONS,
                           canvas_component='map.html')
    
@app.route('/calibration')
def calibration():
    return render_template('calibration.html',
                           title='Calibration',
                           name=PRODUCT_NAME,
                           options=OPTIONS,
                           canvas_component='explanation.html')
    
@app.route('/dynamics')
def dynamics():
    return render_template('dynamics.html',
                           title="Dynamics",
                           name=PRODUCT_NAME,
                           options=OPTIONS,
                           canvas_component='explanation.html')

###########
# Actions #
###########

@app.route('/arm_angle', methods = ['POST'])
def arm_angle():
    if request.method == 'POST':
        input = float(request.form.getlist('number')[0])
        node.arm_send_goal(input)
        return redirect("/")
    
@app.route('/arm_speed', methods = ['POST'])
def arm_speed():
    if request.method == 'POST':
        input = float(request.form.getlist('number')[0])
        node.change_arm_speed(input)
        return redirect("/")
    
@app.route('/table_angle', methods = ['POST'])
def table_angle():
    if request.method == 'POST':
        input = float(request.form.getlist('number')[0])
        node.table_send_goal(input)
        return redirect("/")
    
@app.route('/table_speed', methods = ['POST'])
def table_speed():
    if request.method == 'POST':
        input = float(request.form.getlist('number')[0])
        node.change_table_speed(input)
        return redirect("/")
    
@app.route('/calibrate', methods = ['POST'])
def calibrate():
    if request.method == 'POST':
        if request.form['start'] == 'Start':
            elevation = request.form['elevation']
            azimuth = request.form['azimuth']
            node.arm_send_goal(math.radians(float(elevation)))
            node.table_send_goal(math.radians(float(azimuth)))
            print("Starting machine", file=sys.stderr)
        else:
            print("Stopping machine", file=sys.stderr)
        return redirect("/calibration")
    
@app.route('/simulate', methods = ['POST'])
def calculate():
    if request.method == 'POST':
        geographical_latitude = math.radians(float(request.form.getlist('geographical_latitude')[0]))
        geographical_longitude = math.radians(float(request.form.getlist('geographical_longitude')[0]))
        local_clock_time = float(request.form.getlist('local_clock_time')[0])
        day = float(request.form.getlist('day')[0])
        equation_of_time = get_equation_of_time(day)
        true_solar_time = get_true_solar_time(local_clock_time, geographical_longitude, equation_of_time)
        hour_angle = get_hour_angle(true_solar_time)
        sun_declination = get_sun_declination(day)
        sun_elevation_angle = get_sun_elevation_angle(geographical_latitude, sun_declination, hour_angle)
        sun_azimuth_angle = get_sun_azimuth_angle(sun_elevation_angle, geographical_latitude, sun_declination, true_solar_time)
        node.arm_send_goal(math.degrees(sun_elevation_angle))
        node.table_send_goal(math.degrees(sun_azimuth_angle))
        return f"""<h1>Starting temporary algorithm!!!</h1>
                    <h2>Calculated Angles:</h2>
                    <h3>Sun Elevation Angle (Arm) = {math.degrees(sun_elevation_angle)}</h3>
                    <h3>Sun Azimuth Angle (Table) = {math.degrees(sun_azimuth_angle)}</h3>"""
    
def main():
    # Initialize threads
    webThread = threading.Thread(target=app.run(debug=True), name='webThread')
    nodeThread = threading.Thread(target=rclpy.spin(node), name='nodeThread')

    # Start threads
    webThread.start()
    nodeThread.start()

    # Join threads
    webThread.join()
    nodeThread.join()

if __name__ == '__main__':
    main()

