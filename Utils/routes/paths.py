from flask import render_template, jsonify, request
from flask import Blueprint
import os
import json


paths_bp=Blueprint('paths',__name__)

CONFIG_FILE="data/configs.json"


def start_process():
    return current_app.config["START_PROCESS"]

def stop_process():
    return current_app.config["STOP_PROCESS"]



@paths_bp.route("")        
@paths_bp.route("/home")
def path_page():
    return render_template("paths.html")
    
@paths_bp.route("/addpath")
def path_add():
    return render_template("add_path.html")

@paths_bp.route("/view_path")
def view_path():
    return render_template("view_path.html")  
    
    
@paths_bp.route("/save_config", methods=["POST"])
def save_config():

    new_data = request.json
    print(CONFIG_FILE)
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
        else:
            data = []

        # prevent duplicate path names
        for d in data:
            if d["name"] == new_data["name"]:
                return jsonify({
                    "status":"error",
                    "message":"Path name already exists"
                })

        data.append(new_data)

        with open(CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status":"saved"})

    except Exception as e:
        print(e)
        return jsonify({"status":"error"})

@paths_bp.route("/get_configs")
def get_configs():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            return jsonify(json.load(f))
    return jsonify([])

@paths_bp.route("/delete_config", methods=["POST"])
def delete_config():

    name = request.json.get("name")

    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            data = json.load(f)
    else:
        return jsonify({"status":"error"})

    data = [d for d in data if d["name"] != name]

    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=4)

    return jsonify({"status":"deleted"})
    
#----------------------start oeration and  stop---------------------
 
DATA_FOLDER = "data"
FILE_PATH = os.path.join(DATA_FOLDER, "path_points.json")

@paths_bp.route("/save_data_points", methods=["POST"])
def save_data_points():

    data = request.json

    try:
        with open(FILE_PATH, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status": "saved"})

    except Exception as e:
        print(e)
        return jsonify({"status": "error"})

@paths_bp.route("/startop")
def startop():
    return start_process("path_follow")


@paths_bp.route("/stopop")
def stopop():
    return stop_process("path_follow")
    
    


