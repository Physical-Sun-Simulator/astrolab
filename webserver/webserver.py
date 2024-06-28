# Libraries
from flask import Flask, request, render_template, redirect, flash
from astrolab.user_interface_node import user_interface_node
import rclpy, math, threading, datetime, sys, json, time, os

# Constants
PRODUCT_NAME = "αstrolaβ"
ABOUT_LINK = "https://github.com/Physical-Sun-Simulator"
ABOUT_DESCRIPTION = "© Aqiel Oostenbrug - Learn More"
OPTIONS = ["simulation", "calibration", "dynamics"]
SIMULATE_MSG = "Starting simulation"
CALIBRATE_MSG = "Starting calibration"
MOVE_MSG = "Starting dynamic lighting"
ABORT_MSG = "Aborting current job"
CONFIGURATION_INTERVAL = 0.5
FLASK_CONFIG_PATH = "config.json"

# Initialize Flask
app = Flask(__name__)
app.config.from_file(FLASK_CONFIG_PATH, load=json.load)

# Initialize rclpy
rclpy.init(args=None)

# Global variables
node = user_interface_node()

# Simplification functions
get_angle = lambda input: float(request.form[input])
start_pressed = lambda: request.form["button"] == "Start"
stop_pressed = lambda: request.form["button"] == "Stop"
get_datetime = lambda input, format: datetime.datetime.strptime(
    request.form[input], format
)
get_day_number = lambda input: datetime.date.timetuple(
    get_datetime(input, "%Y-%m-%d")
).tm_yday
get_time = (
    lambda input: get_datetime(input, "%H:%M").hour
    + get_datetime(input, "%H:%M").minute / 60.0
    + get_datetime(input, "%H:%M").second / 60.0
)
get_speed = lambda input: float(request.form[input]) / 100.0

############
# Webpages #
############

@app.route("/")
@app.route("/simulation")
def simulation():
    """Handle simulation webpage requests."""
    return render_template(
        "simulation.html",
        title="Simulation",
        name=PRODUCT_NAME,
        options=OPTIONS,
        canvas_component="map.html",
        about_link=ABOUT_LINK,
        about_description=ABOUT_DESCRIPTION,
    )

@app.route("/calibration")
def calibration():
    """Handle calibration webpage requests."""
    return render_template(
        "calibration.html",
        title="Calibration",
        name=PRODUCT_NAME,
        options=OPTIONS,
        canvas_component="explanation.html",
        about_link=ABOUT_LINK,
        about_description=ABOUT_DESCRIPTION,
    )

@app.route("/dynamics")
def dynamics():
    """Handle dynamic lighting webpage requests."""
    return render_template(
        "dynamics.html",
        title="Dynamics",
        name=PRODUCT_NAME,
        options=OPTIONS,
        canvas_component="explanation.html",
        about_link=ABOUT_LINK,
        about_description=ABOUT_DESCRIPTION,
    )


@app.route("/configuration")
def configuration():
    """Handle configuration requests."""
    configuration = {"elevation": node.get_elevation(), "azimuth": node.get_azimuth()}
    configuration_json = json.dumps(configuration)

    return configuration_json

###########
# Actions #
###########

@app.route("/simulate", methods=["POST"])
def simulate():
    """Handle simulation form submissions."""
    if request.method == "POST":
        if start_pressed():
            latitude = get_angle("latitude")
            longitude = get_angle("longitude")
            day = get_day_number("day")
            time = get_time("time")
            threading.Thread(
                target=node.simulate, args=(latitude, longitude, day, time)
            ).start()
            flash(SIMULATE_MSG)
    return redirect("/simulation")

@app.route("/calibrate", methods=["POST"])
def calibrate():
    """Handle calibration form submissions."""
    if request.method == "POST":
        if start_pressed():
            elevation = get_angle("elevation")
            azimuth = get_angle("azimuth")
            threading.Thread(target=node.calibrate, args=(elevation, azimuth)).start()
            flash(CALIBRATE_MSG)
    return redirect("/calibration")

@app.route("/move", methods=["POST"])
def move():
    """Handle dynamic lighting form submissions."""
    if request.method == "POST":
        if start_pressed():
            elevation_one = get_angle("elevation-one")
            azimuth_one = get_angle("azimuth-one")
            elevation_two = get_angle("elevation-two")
            azimuth_two = get_angle("azimuth-two")
            speed = get_speed("speed")
            threading.Thread(
                target=node.move,
                args=(elevation_one, azimuth_one, elevation_two, azimuth_two, speed),
            ).start()
            flash(MOVE_MSG)
    return redirect("/dynamics")


@app.route("/stop", methods=["POST"])
def stop():
    """Stop current job."""
    node.abort()
    flash(ABORT_MSG)
    return redirect(request.referrer)

#########
# Setup #
#########

def main():
    """First function to be executed."""
    # Initialize threads
    nodeThread = threading.Thread(target=rclpy.spin, name="node_thread", args=(node,))
    webThread = threading.Thread(
        target=app.run,
        name="web_thread",
        kwargs={"debug": False, "use_reloader": False},
    )

    # Start threads
    webThread.start()
    nodeThread.start()

    # Join threads
    webThread.join()
    nodeThread.join()

if __name__ == "__main__":
    main()
