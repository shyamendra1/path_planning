from flask import  jsonify, request, render_template
from flask import Blueprint
import os
import json


tractor_bp=Blueprint('tractor',__name__)
TRACTOR_FILE = "data/tractor.json"


@tractor_bp.route("")
@tractor_bp.route("/home")
def tractor():
    return render_template("tractor.html")

@tractor_bp.route("/addtractor")
def addtractor():
    return render_template("add_tractor.html")
   


@tractor_bp.route("/get_tractors")
def get_tractors():

    if os.path.exists(TRACTOR_FILE):
        with open(TRACTOR_FILE, "r") as f:
            data = json.load(f)
    else:
        data = []

    return jsonify(data)

@tractor_bp.route("/save_tractor", methods=["POST"])
def save_tractor():

    new_data = request.json

    name = str(new_data.get("name", "")).strip()
    wheelbase = new_data.get("wheelbase")
    radius = new_data.get("radius")

    # -------- VALIDATION ----------
    if not name or wheelbase is None or radius is None:
        return jsonify({"status":"error", "message":"All fields are required"})

    try:
        wheelbase = float(wheelbase)
        radius = float(radius)
    except:
        return jsonify({"status":"error", "message":"Wheelbase and radius must be numbers"})

    if wheelbase <= 0 or radius <= 0:
        return jsonify({"status":"error", "message":"Values must be positive"})

    try:
        # Load existing data
        if os.path.exists(TRACTOR_FILE):
            with open(TRACTOR_FILE, "r") as f:
                data = json.load(f)
        else:
            data = []

        # -------- CHECK DUPLICATE NAME ----------
        for tractor in data:
            if tractor["name"].lower() == name.lower():
                return jsonify({
                    "status":"error",
                    "message":"Tractor with same name already exists"
                })

        # Append new tractor
        data.append({
            "name": name,
            "wheelbase": wheelbase,
            "radius": radius
        })

        # Save file
        with open(TRACTOR_FILE, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status":"saved"})

    except Exception as e:
        print(e)
        return jsonify({"status":"error", "message":"Server error"})

@tractor_bp.route("/delete_tractor", methods=["POST"])
def delete_tractor():

    name = request.json.get("name")

    try:
        if os.path.exists(TRACTOR_FILE):
            with open(TRACTOR_FILE, "r") as f:
                data = json.load(f)
        else:
            return jsonify({"status":"error"})

        # remove tractor
        data = [t for t in data if t["name"] != name]

        with open(TRACTOR_FILE, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status":"deleted"})

    except Exception as e:
        print(e)
        return jsonify({"status":"error"})
