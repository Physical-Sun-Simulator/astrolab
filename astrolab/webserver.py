from flask import Flask, request, render_template, redirect
from astrolab.user_interface_node import user_interface_node
import rclpy

app = Flask(__name__)

rclpy.init(args=None)
node = user_interface_node()

@app.route("/")
def index():
    return """<form action="/angle" method="POST" class="combo-box">
                <input type="text" name="number" placeholder="Insert an angle">
                <input type="submit" value="submit" />
            </form>"""

@app.route('/angle', methods = ['POST'])
def angle():
    if request.method == 'POST':
        input = float(request.form.getlist('number')[0])
        rclpy.spin_once(node,timeout_sec=1.0)
        node.arm_send_goal(input)
        return redirect("/")
    
def main():
    app.run(debug=True)

if __name__ == '__main__':
    main()

