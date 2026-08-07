from flask import render_template, jsonify, request
from flask import Blueprint
import os
import json
from Utils.helper.geodesy import Geodesy

farm_bp=Blueprint('farm',__name__)
FARM_FILE = "data/farms.json"

@farm_bp.route("")        
@farm_bp.route("/home")
def farm():
    return render_template("farms.html")

@farm_bp.route("/addfarms")
def farms_list():
    return render_template("add_farm.html")  


@farm_bp.route("/get_farms")
def get_farms():
    if os.path.exists(FARM_FILE):
        with open(FARM_FILE, "r") as f:
            return jsonify(json.load(f))
    return jsonify([])

@farm_bp.route("/curr_loc")
def get_loc():
    try:
        data = data_read.last_data()
        #print(data)
        if data:
            #print(data)
            return jsonify(data)
        else:
            return jsonify([])
    except Exception as e:
        print(f"Error in stream: {e}")


@farm_bp.route("/delete_farm", methods=["POST"])
def delete_farm():

    name = request.json.get("name")

    try:
        if os.path.exists(FARM_FILE):
            with open(FARM_FILE, "r") as f:
                data = json.load(f)
        else:
            return jsonify({"status":"error"})

        data = [f for f in data if f["name"] != name]

        with open(FARM_FILE, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status":"deleted"})

    except:
        return jsonify({"status":"error"})
    
@farm_bp.route("/save_farm", methods=["POST"])
def save_farm():

    new_farm = request.json
    #print("farm data")
    #print(new_farm.get('boundary'))
    #check_shape=True
    check_shape = Geodesy.verify_convex(new_farm.get('boundary'))
    if (check_shape):
        
        try:
            if os.path.exists(FARM_FILE):
                with open(FARM_FILE, "r") as f:
                    data = json.load(f)
            else:
                data = []

            # prevent duplicate name
            for farm in data:
                if farm["name"] == new_farm["name"]:
                    return jsonify({"status":"error","message":"Farm already exists"})

            data.append(new_farm)

            with open(FARM_FILE, "w") as f:
                json.dump(data, f, indent=4)

            return jsonify({"status":"saved"})

        except Exception as e:
            print(e)
            return jsonify({"status":"error"})
    else:
        return jsonify({"status" : "error","message":"shape cannot be concave."})

@farm_bp.route("/view_farm")
def view_farm():
    return render_template("view_farm.html")
