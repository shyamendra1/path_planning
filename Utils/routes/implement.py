from flask import render_template, jsonify, request
from flask import Blueprint
import os
import json


implement_bp=Blueprint('implement',__name__)

IMPLEMENT_FILE = "data/implement.json"

@implement_bp.route("")
@implement_bp.route("/home")
def implement():
    return render_template("implement.html")

@implement_bp.route("/addimplement")
def implement_list():
    return render_template("add_implement.html")  


@implement_bp.route("/get_implements")
def get_implements():

    if os.path.exists(IMPLEMENT_FILE):
        with open(IMPLEMENT_FILE, "r") as f:
            data = json.load(f)
    else:
        data = []

    return jsonify(data)
    
@implement_bp.route("/save_implement", methods=["POST"])
def save_implement():

    new_data = request.json

    type_ = str(new_data.get("type", "")).strip()
    name = str(new_data.get("name", "")).strip()
    width = new_data.get("width")

    # -------- VALIDATION ----------
    if not type_ or not name or width is None:
        return jsonify({"status":"error", "message":"All fields are required"})

    try:
        width = float(width)
    except:
        return jsonify({"status":"error", "message":"Width must be a number"})

    if width <= 0:
        return jsonify({"status":"error", "message":"Width must be positive"})

    try:
        # Load existing data
        if os.path.exists(IMPLEMENT_FILE):
            with open(IMPLEMENT_FILE, "r") as f:
                data = json.load(f)
        else:
            data = []

        # -------- DUPLICATE CHECK ----------
        for imp in data:
            if imp["name"].lower() == name.lower():
                return jsonify({
                    "status":"error",
                    "message":"Implement with same name already exists"
                })

        # Save new implement
        data.append({
            "type": type_,
            "name": name,
            "width": width
        })

        with open(IMPLEMENT_FILE, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status":"saved"})

    except Exception as e:
        print(e)
        return jsonify({"status":"error", "message":"Server error"})    

@implement_bp.route("/delete_implement", methods=["POST"])
def delete_implement():

    name = request.json.get("name")

    try:
        if os.path.exists(IMPLEMENT_FILE):
            with open(IMPLEMENT_FILE, "r") as f:
                data = json.load(f)
        else:
            return jsonify({"status":"error"})

        # remove implement
        data = [i for i in data if i["name"] != name]

        with open(IMPLEMENT_FILE, "w") as f:
            json.dump(data, f, indent=4)

        return jsonify({"status":"deleted"})

    except Exception as e:
        print(e)
        return jsonify({"status":"error"})        


