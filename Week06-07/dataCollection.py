# Save and label images for training your neural network
# You may move the robot around by dragging it inside Gazebo 
# or by running your keyboard teleoperation function

import csv
import cv2
import os
import sys

import penguinPiC
ppi = penguinPiC.PenguinPi()

# save images and a csv file with each row being [image_label,image_file_name]
class DatasetWriter:
    def __init__(self, annot, image_count):
        self.folder = 'dataset/'
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
        
        img_fname = self.folder + "labels.csv"
        self.img_f = open(img_fname, 'a')
        self.img_fc = csv.writer(self.img_f)
    
    def __del__(self):
        self.img_f.close()
    
    def write_image(self):
        labels = ['sheep','coke','neither']
        img_fname = self.folder + str(image_count)+".png"
        row = [labels[annot], img_fname]

        self.img_fc.writerow(row)
        self.img_f.flush()

        image = ppi.get_image()
        cv2.imwrite(img_fname, image)


# get inputs for labeling
def inputNumber(message):
    while True:
        try:
            userInput = int(input(message))       
        except ValueError:
            print("Not an integer! Try again.")
            continue
        else:
            return userInput 
            break 


# main program
if __name__ == "__main__":
    # collect data
    images_to_collect = inputNumber('Please enter how many images you would like to collect:\n')
    for i in range(images_to_collect):
        input('Press ENTER to capture an image.')
        image_count = i
        annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
        data = DatasetWriter(annot, image_count)
        data.write_image()
        print('Collected image No.', i)

    print('Finished data collection.\n')


