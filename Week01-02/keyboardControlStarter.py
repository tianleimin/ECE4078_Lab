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

<<<<<<< HEAD

=======
        # adding 'B' key for boost functionality
        # can be toggled on and off
>>>>>>> 10f86b73417743093e7a2b3d0fbb7145e238558f
        elif str(key) == "'b'":
            if (self.wheel_vel_forward == 100):
                self.wheel_vel_forward = 150
                self.wheel_vel_rotation = 30
            else:
                self.wheel_vel_forward = 100
                self.wheel_vel_rotation = 20

<<<<<<< HEAD


=======
        # space key for stopping the bot
>>>>>>> 10f86b73417743093e7a2b3d0fbb7145e238558f
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
            left_speed  -= self.wheel_vel_rotation
            right_speed += self.wheel_vel_rotation

        if self.directions[3]:
            left_speed  += self.wheel_vel_rotation
            right_speed -= self.wheel_vel_rotation

        return left_speed, right_speed
        
    
    def send_drive_signal(self):
        if not self.ppi is None:
            lv, rv = self.get_drive_signal()
            lv, rv = self.ppi.set_velocity(lv, rv)
            self.wheel_vels = [lv, rv]
            print(self.wheel_vels)
            
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

        # different font colours for 
        # on screen text
        font_col_1 = (255, 255, 255)
        font_col_2 = (0, 0, 255)
        font_col_3 = (0, 255, 0)
        font_col_4 = (255, 0, 0)

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

        # Boost flag for display
        if L_Wvel > 120 or R_Wvel > 120:
            BOOST_FLAG = "Send mode"
        else:
            BOOST_FLAG = "OFF"
        
        # Direction flag for display
        direction = ''
        if (L_Wvel == R_Wvel) & ((L_Wvel + R_Wvel) > 0):
            direction = 'Forward'
        elif (L_Wvel == R_Wvel) & ((L_Wvel + R_Wvel) < 0):
            direction = 'Backward'
        elif L_Wvel < R_Wvel:
            direction = 'Turn Left'
        elif L_Wvel > R_Wvel:
            direction = 'Turn Right'
        elif L_Wvel == R_Wvel == 0:
            direction = 'Stop'


        # GUI texts
        cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col_1, line_type)
        cv2.putText(resized, 'Direction : ' + direction, (15, 550), font, font_scale, font_col_4, line_type)
        cv2.putText(resized, 'Wheel Velocity : ' + str((L_Wvel + R_Wvel)/2), (15, 595), font, font_scale, font_col_3, line_type)
        cv2.putText(resized, 'Left_W: ' + str(L_Wvel), (15, 630), font, 0.75, font_col_3, line_type)
        cv2.putText(resized, 'Right_W: ' + str(R_Wvel), (15, 660), font, 0.75, font_col_3, line_type)
        cv2.putText(resized, 'BOOST: ' + str(BOOST_FLAG), (15, 700), font, 1, font_col_2, line_type)

        cv2.imshow('video', resized)
        cv2.waitKey(1)

        continue
