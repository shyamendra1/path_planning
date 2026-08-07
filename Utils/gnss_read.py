import os
import json
class GNSSData:
    def __init__(self, filepath):
        """
        :param filepath - path to data file
        """
        self.filepath = filepath
        

    def last_data(self):
        if not os.path.exists(self.filepath):
            print("The specified path does not exist")
            return None

        try:
            with open(self.filepath, 'r') as file:
                data=json.load(file)

            return {"latitude": data["Latitude"], "longitude": data["Longitude"], "Head":data["Head"], "Speed":data["Speed"], "Quality":data["Quality"], "Differential Age":data["Differential Age"], "Number of Satellite":data["Number of Satellite"],"NTRIP":data["NTRIP"],"Time":data["Time"]}

        except Exception as e:
            pass
            print(f"Error reading GPS data: {e}")

        return None

