import csv
import json
import os
import time
import socket
from threading import Lock

from flask import Flask, request, render_template, jsonify, Response

from Utils.gnss_read import GNSSData
from Utils.generate_path import Path_plan
from Utils.run import Run_process


from Utils.routes.tractor import tractor_bp
from Utils.routes.implement import implement_bp
from Utils.routes.farm import farm_bp
from Utils.routes.ntrip import ntrip_bp
from Utils.routes.paths import paths_bp
from Utils.routes.gnssdata import gnssdash_bp
# ---------------- CONFIG ----------------

PROCESS_CONFIG_FILE = "data/process_config.json"

gnss_data="../cors_ntrip/dev.json"
save_path = "data/path_points.json"

# ----------------------------------------
#design flask application
app = Flask(__name__)

#-----------tractor page----------------------
app.register_blueprint(tractor_bp,url_prefix='/tractor')
#------------------implement page------------------
app.register_blueprint(implement_bp,url_prefix='/implement')
#-----------------------------farm--------------------------------------        
app.register_blueprint(farm_bp,url_prefix='/farm')
#--------------------------ntrip client------------------------
app.register_blueprint(ntrip_bp,url_prefix='/ntrip')
#------------------------path--------------------------------
app.register_blueprint(paths_bp, url_prefix="/paths")
#-----------------------gnss_dashboard---------------
app.register_blueprint(gnssdash_bp, url_prefix="/gnssdata")



#-----------------------------------------------
data_read = GNSSData(gnss_data)
gcp = []

# Load process config
with open(PROCESS_CONFIG_FILE, "r") as f:
    PROCESS_CONFIG = json.load(f)

# Process pool + lock
process_pool = {}
process_lock = Lock()


# ---------------- PROCESS MANAGER ----------------

def get_runner(name, extra_args=None):
    cfg = PROCESS_CONFIG.get(name)

    if not cfg:
        raise Exception(f"No config for process: {name}")

    args = cfg.get("args", []).copy()

    if extra_args:
        args.extend(extra_args)

    return Run_process(
        path=cfg["path"],
        proc_name=cfg["proc_name"],
        password=None,
        args=args
    )

def is_running(runner):
    return runner and runner.process and runner.process.poll() is None


# ---------------- GENERIC APIs ----------------

@app.route("/start/<proc_name>")
def start_process(proc_name):
    with process_lock:
        runner = process_pool.get(proc_name)

        if is_running(runner):
            return jsonify({"status": "already running"})

        runner = get_runner(proc_name)
        runner.runobject()

        process_pool[proc_name] = runner

        return jsonify({"status": "started"})


@app.route("/stop/<proc_name>")
def stop_process(proc_name):
    with process_lock:
        runner = process_pool.get(proc_name)

        if not runner:
            return jsonify({"status": "not running"})

        msg = runner.stopobject()
        process_pool.pop(proc_name, None)

        return jsonify({"status": msg})


@app.route("/status/<proc_name>")
def status(proc_name):
    runner = process_pool.get(proc_name)

    if is_running(runner):
        return jsonify({"status": "running"})
    else:
        return jsonify({"status": "stopped"})

app.config["GET_RUNNER"] = get_runner
app.config["PROCESS_POOL"] = process_pool
app.config["PROCESS_LOCK"] = process_lock
app.config["START_PROCESS"] = start_process
app.config["STOP_PROCESS"] = stop_process
# ---------------- LIVE GNSS STREAM ----------------

def event_stream():
    while True:
        time.sleep(0.5)
        try:
            data = data_read.last_data()
            #print(data)
            yield f"data: {json.dumps(data if data else {})}\n\n"
        except Exception as e:
            print("Stream error:", e)
            break


@app.route("/livelocation")
def livelocation():
    return Response(event_stream(), mimetype="text/event-stream")

@app.route('/stop', methods=['GET'])
def stop():
    res = {"response":"stopped"}
    return jsonify(res)

# ---------------- UI ROUTES ----------------

@app.route("/")
@app.route("/check")
def checkpage():
    return render_template("check.html")


@app.route("/home")
def home():
    return render_template("homepage.html")

# ---------------- SYSTEM CHECK ----------------

@app.route("/check/<test>")
def check(test):

    if test == "steering":
        return jsonify({"status": "ok"})

    elif test == "ntrip":
        timeout = time.time() + 2  

        while time.time() < timeout:
            data = data_read.last_data()
            if data and data.get("Quality") == 4:
                return jsonify({"status": "ok"})

        return jsonify({"status": "fail"})

    return jsonify({"status": "fail"})


# ---------------- GNSS (WRAPPER) ----------------

@app.route("/start_gnss")
def start_gnss():
    return start_process("gnss")


@app.route("/stop_gnss")
def stop_gnss():
    return stop_process("gnss")


# ---------------- PATH PLANNING ----------------
@app.route('/path', methods=['POST'])
def path():
    dat = request.json
    # print(dat)
    field_name = dat["farm"]["name"]
    boundary = dat["farm"]["boundary"]
    width = dat['implement']['width']
    radius = dat['tractor']['radius']
    wheelbase = dat['tractor']['wheelbase']

    gcp = []
    for n in range(len(boundary)):
        gcp.append([[boundary[n - 1][0], boundary[n - 1][1]],
                    [boundary[n][0], boundary[n][1]]])


    out = Path_plan(gcp, save_path, width, radius, wheelbase, field_name)
    tp,head= out.path()

    return jsonify({"track":tp ,"head":head})
  
# ---------------- MAIN ----------------

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=True)
