# Libraries
from flask import Flask, request, render_template, redirect, flash
from astrolab.user_interface_node import user_interface_node
import rclpy, math, threading, datetime, sys, json, time, os

# Initialize Flask
app = Flask(__name__)
app.secret_key = b'test'

# Initialize rclpy
rclpy.init(args=None)

# Global variables
node = user_interface_node()

# Constants
PRODUCT_NAME = "αstrolaβ"
OPTIONS = ['simulation', 'calibration', 'dynamics']
SIMULATE_MSG = 'Starting simulation'
CALIBRATE_MSG = 'Starting calibration'
MOVE_MSG = 'Starting dynamic lighting'
ABORT_MSG = 'Aborting current job'
CONFIGURATION_INTERVAL = 0.5
BASE_DIRECTORY = os.path.abspath(os.path.dirname(__file__))
CONFIGURATION_PATH = os.path.join(BASE_DIRECTORY, "data/configuration.json")

# Simplification functions
get_rad = lambda input: math.radians(float(request.form[input]))
start_pressed = lambda: request.form['button'] == 'Start'
stop_pressed = lambda: request.form['button'] == 'Stop'
get_datetime = lambda input, format: datetime.datetime.strptime(request.form[input], format)
get_day_number = lambda input: datetime.date.timetuple(get_datetime(input, '%Y-%m-%d')).tm_yday
get_time = lambda input: get_datetime(input, '%H:%M').hour + get_datetime(input, '%H:%M').minute / 60.0 + get_datetime(input, '%H:%M').second / 60.0
get_speed = lambda input: float(request.form[input]) / 100.0

############
# Webpages #
############

@app.route("/")
@app.route('/simulation')
def simulation():
    """ Handle simulation webpage requests. """
    return render_template('simulation.html',
                           title="Simulation",
                           name=PRODUCT_NAME,
                           options=OPTIONS,
                           canvas_component='map.html')
    
@app.route('/calibration')
def calibration():
    """ Handle calibration webpage requests. """
    return render_template('calibration.html',
                           title='Calibration',
                           name=PRODUCT_NAME,
                           options=OPTIONS,
                           canvas_component='explanation.html')
    
@app.route('/dynamics')
def dynamics():
    """ Handle dynamic lighting webpage requests. """
    return render_template('dynamics.html',
                           title="Dynamics",
                           name=PRODUCT_NAME,
                           options=OPTIONS,
                           canvas_component='explanation.html')

@app.route('/configuration')
def configuration():
    """ Handle configuration requests. """   
    # Read configuration json
    with open(CONFIGURATION_PATH, 'r') as file:
        configuration_json = json.load(file)
    
    return configuration_json

###########
# Actions #
###########

@app.route('/simulate', methods = ['POST'])
def simulate():
    """ Handle simulation form submissions. """
    if request.method == 'POST':
        if start_pressed():
            latitude = float(request.form['latitude'])
            longitude = float(request.form['longitude'])  
            day = get_day_number('day')
            time = get_time('time')
            node.simulate(latitude, longitude, day, time)
            # TODO: Add calculated elevation and azimuth angle to simulation msg
            flash(SIMULATE_MSG)
    return redirect("/simulation")

@app.route('/calibrate', methods = ['POST'])
def calibrate():
    """ Handle calibration form submissions. """
    if request.method == 'POST':
        if start_pressed():
            elevation = float(request.form['elevation'])
            azimuth = float(request.form['azimuth'])
            node.calibrate(elevation, azimuth)
            flash(CALIBRATE_MSG)
    return redirect("/calibration")
    
@app.route('/move', methods = ['POST'])
def move():
    """ Handle dynamic lighting form submissions. """
    if request.method == 'POST':
        if start_pressed():
            elevation_one = get_rad('elevation-one')
            azimuth_one = get_rad('azimuth-one')
            elevation_two = get_rad('elevation-two')
            azimuth_two = get_rad('azimuth-two')
            speed = get_speed(request.form['speed'])
            node.move(elevation_one, azimuth_one, elevation_two, azimuth_two, speed)
            flash(MOVE_MSG)
    return redirect("/move")

@app.route('/stop', methods = ['POST'])
def stop():
    """ Stop current job. """
    node.abort()
    flash(ABORT_MSG)
    return redirect(request.referrer)

###############
# Inaccesible #
###############

def update_configuration():
    """ Update configuration information. """
    configuration = {
        "elevation": node.get_elevation(),
        "azimuth": node.get_azimuth()
    }
    
    configuration_json = json.dumps(configuration)
    
    with open(CONFIGURATION_PATH, "w") as file:
        file.write(configuration_json)
    
def main():
    """ First function to be executed. """
    # Initialize threads
    nodeThread = threading.Thread(target=rclpy.spin, name='node_thread', args=(node,))
    webThread = threading.Thread(target=app.run, name='web_thread', kwargs={"debug":False, 'use_reloader':False})
    node.create_timer(CONFIGURATION_INTERVAL, update_configuration)
    
    # Start threads
    webThread.start()
    nodeThread.start()
    

    # Join threads
    webThread.join()
    nodeThread.join()

if __name__ == '__main__':
    main()

