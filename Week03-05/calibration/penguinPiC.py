# access each wheel and the camera onboard of PenguinPi

import numpy as np
import requests
import cv2      

class PenguinPi:
    def __init__(self, ip = 'localhost'):
        self.ip = ip
        self.port = 40000

    def set_velocity(self, lvel, rvel, time=0):
        if time == 0:
            r = requests.get(f"http://{self.ip}:{self.port}/robot/set/velocity?value="+str(lvel)+","+str(rvel))
        else:
            assert (time > 0), "Time must be positive."
            assert (time < 30), "Time must be less than network timeout (20s)."
            r = requests.get("http://"+self.ip+":"+str(self.port)+"/robot/set/velocity?value="+str(lvel)+","+str(rvel)
                            +"&time="+str(time))
        return lvel, rvel
        
    def get_image(self):
        try:
            r = requests.get(f"http://{self.ip}:{self.port}/camera/get")
            img = cv2.imdecode(np.frombuffer(r.content,np.uint8), cv2.IMREAD_COLOR)
        except (requests.exceptions.ConnectTimeout, requests.exceptions.ConnectionError, requests.exceptions.ReadTimeout) as e:
            print("Image retrieval timed out.")
            img = np.zeros((240,320,3), dtype=np.uint8)
        return img
            
