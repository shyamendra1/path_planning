import socket
from flask import Blueprint, request, jsonify, current_app, render_template

ntrip_bp = Blueprint("ntrip", __name__)

# ---------------- HELPERS ----------------

def get_runner():
    return current_app.config["GET_RUNNER"]

def process_pool():
    return current_app.config["PROCESS_POOL"]

def process_lock():
    return current_app.config["PROCESS_LOCK"]

def stop_process():
    return current_app.config["STOP_PROCESS"]

# ---------------- ROUTES ----------------
@ntrip_bp.route("")
@ntrip_bp.route("/home")
def ntripcl():
    return render_template("ntrip_client.html")

@ntrip_bp.route("/get_mountpoints", methods=["POST"])
def get_mountpoints():
    data = request.json
    host = data["host"]
    port = int(data["port"])

    try:
        s = socket.socket()
        s.settimeout(5)
        s.connect((host, port))

        s.send(b"GET / HTTP/1.1\r\nNTRIP-Version:Ntrip/1.0\r\nUser-Agent: NTRIP\r\n\r\n")
        response = s.recv(4096).decode(errors="ignore")

        mountpoints = [
            line.split(";")[1]
            for line in response.split("\n")
            if line.startswith("STR")
        ]

        s.close()

        return jsonify({
            "status": "success",
            "mountpoints": mountpoints
        })

    except Exception as e:
        return jsonify({
            "status": "error",
            "mountpoints": [],
            "error": str(e)
        })


@ntrip_bp.route("/connect_ntrip", methods=["POST"])
def connect_ntrip():
    '''data = request.json

    extra_args = [
        data["host"],
        str(data["port"]),
        data["mountpoint"],
        data["username"],
        data["password"]
    ]

    with process_lock():
        runner = get_runner()("ntrip", extra_args)
        runner.runobject()

        process_pool()["ntrip"] = runner'''

    return jsonify({"status": "connected"})


@ntrip_bp.route("/disconnect_ntrip", methods=["GET"])
def disconnect_ntrip():
    return stop_process()("ntrip")


@ntrip_bp.route("/status", methods=["GET"])
def status_ntrip():
    runner = process_pool().get("ntrip")

    if runner and runner.process and runner.process.poll() is None:
        return jsonify({"status": "running"})
    else:
        return jsonify({"status": "stopped"})
