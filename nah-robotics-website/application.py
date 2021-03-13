from flask import Flask, render_template, request

app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


@app.route('/favicon.ico')
def favicon():
    return render_template("index.html")


@app.route("/skystone")
def skystone2020():
    return render_template("skystone.html")


@app.route("/misc2020")
def misc2020():
    return render_template("misc2020.html")


@app.route("/practice2020")
def practice2020():
    return render_template("practice2020.html")


@app.route("/programming2020")
def programming2020():
    return render_template("programming2020.html")


@app.route("/robot2020")
def robot2020():
    return render_template("robot2020.html")


@app.route("/tournaments2020")
def tournaments2020():
    return render_template("tournaments2020.html")


@app.route("/game_mechanics2020")
def game_mechanics2020():
    return render_template("game_mechanics2020.html")