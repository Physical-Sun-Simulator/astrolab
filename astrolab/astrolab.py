from flask import Flask, request, render_template, redirect


app = Flask(__name__)

@app.route("/")
def hello_world():
    return """<form action="/angle" method="POST" class="combo-box">
                <input type="text" name="number" placeholder="Insert an angle">
                <input type="submit" value="submit" />
            </form>"""

@app.route('/angle', methods = ['POST'])
def angle():
    if request.method == 'POST':
        input = request.form.getlist('number')[0]
        print(input)
        return redirect("/")
    
def main():
    app.run(debug=True)

if __name__ == '__main__':
    main()
