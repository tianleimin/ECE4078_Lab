# teleoperate the robot through keyboard control
# getting-started code

from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np

class Keyboard:
    
    

    def __init__(self, ppi=None):
        # feel free to change the speed, or add keys to do so
        self. wheel_vel_forward = 100
        self.wheel_vel_rotation = 20
        
        # storage for key presses
        self.directions = [False for _ in range(4)]
        self.signal_stop = False 

        # connection to PenguinPi robot
        self.ppi = ppi
        self.wheel_vels = [0, 0]

        self.listener = Listener(on_press=self.on_press).start()

    def on_press(self, key):
        print(key)
        # use arrow keys to drive, space key to stop
        # feel free to add more keys
        if key == Key.up:
            self.directions = [True, False, False, False]

        elif key == Key.down:
            self.directions = [False, True, False, False]

        elif key == Key.left:
            self.directions[2] = True
            self.directions[3] = False

        elif key == Key.right:
            self.directions[3] = True
            self.directions[2] = False

        elif key == Key.space:
            self.directions[:] = [False, False, False, False]

        self.send_drive_signal()
        
    def get_drive_signal(self):  
        left_speed  = 0
        right_speed = 0

        # translate the key presses into drive signals
        if self.directions[0]:
            left_speed  = self.wheel_vel_forward
            right_speed = self.wheel_vel_forward
            
        if self.directions[1]:
            left_speed  = -self.wheel_vel_forward
            right_speed = -self.wheel_vel_forward

        if self.directions[2]:
            left_speed  += 0
            right_speed += self.wheel_vel_rotation

        if self.directions[3]:
            left_speed  += self.wheel_vel_rotation
            right_speed += 0

        return left_speed, right_speed
        
    
    def send_drive_signal(self):
        if not self.ppi is None:
            lv, rv = self.get_drive_signal()
            lv, rv = self.ppi.set_velocity(lv, rv)
            self.wheel_vels = [lv, rv]
            print(self.wheel_vels)
            print(self.directions)
            
    def latest_drive_signal(self):
        return self.wheel_vels
    

if __name__ == "__main__":
    import penguinPiC
    ppi = penguinPiC.PenguinPi()

    keyboard_control = Keyboard(ppi)

    cv2.namedWindow('video', cv2.WINDOW_NORMAL);
    cv2.setWindowProperty('video', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE);

    while True:
        # font display options
        font = cv2.FONT_HERSHEY_SIMPLEX
        location = (0, 0)
        font_scale = 1
        font_col = (255, 255, 255)
        line_type = 2

        # get velocity of each wheel
        wheel_vels = keyboard_control.latest_drive_signal()
        L_Wvel = wheel_vels[0]
        R_Wvel = wheel_vels[1]

        # get current camera frame
        curr = ppi.get_image()

        # scale to 144p
        # feel free to change the resolution
        resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

        # feel free to add more GUI texts
        cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)

        cv2.imshow('video', resized)
        cv2.waitKey(1)

        continue
