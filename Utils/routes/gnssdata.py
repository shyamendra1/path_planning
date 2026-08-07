from flask import  jsonify, request, render_template, Response
from flask import Blueprint
import os
import json
import time
from Utils.gnss_read import GNSSData

gnssdash_bp=Blueprint('gnssdata',__name__)

@gnssdash_bp.route("")
@gnssdash_bp.route("/home")
def tractor():
    return render_template("gnssdash.html")

gnss_data="../cors_ntrip/dev.json"
data_read = GNSSData(gnss_data)

def event_stream():
    while True:
        time.sleep(0.7)
        try:
            data = data_read.last_data()
            #print(data)
            yield f"data: {json.dumps(data if data else {})}\n\n"
        except Exception as e:
            print("Stream error:", e)
            break


@gnssdash_bp.route("/livelocation")
def livelocation():
    return Response(event_stream(), mimetype="text/event-stream")    
