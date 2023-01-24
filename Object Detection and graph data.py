import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar
from gpiozero import Robot, Motor
import csv
from array import *
import matplotlib.pyplot as plt

data_arr = []
y = []

# Set up pygame and the display
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
lcd = pygame.display.set_mode((320,240))
pygame.mouse.set_visible(False)
lcd.fill((0,0,0))
pygame.display.update()
 
# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# Pins where motors are connected at (see Motor controller schaltplan for more)
LeftMotor = Motor(17,22,18)
RightMotor = Motor(4,24,19)

# Creating data files scanned by Lidar!
file_Open = open("data.csv", "w")
data = "Data; \n"
file_Open.write(data)
file_Open.close()

        
#pylint: disable=redefined-outer-name,global-statement
def main():
    def process_data(data):
        global max_distance
        lcd.fill((0,0,0))
        for angle in range(360):
            distance = data[angle]
            if distance > 0:                  # ignore initially ungathered data points
                max_distance = max([min([5000, distance]), max_distance])
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
                lcd.set_at(point, pygame.Color(255, 255, 255))
        pygame.display.update()
     
     
    scan_data = [0]*360

    try:
        print(lidar.info)
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])] = distance
            process_data(scan_data)
           
            straight_Point = scan_data[180]
            print(straight_Point)
            if straight_Point > 250 and straight_Point < 600:
                RightMotor.forward(0)
                LeftMotor.forward(0)
                #print(straight_Point)
                length = len(data_arr)
                #print(length)
                for i in range(length):
                    y.append(i)
                print(y)
                # plotting the points
                plt.plot(y, data_arr)

                # naming the x axis
                plt.xlabel('x - axis')
                # naming the y axis
                plt.ylabel('y - axis')

                # giving a title to my graph
                plt.title('Distance')

                # function to show the plot
                plt.show()

            else:
                data_arr.append(straight_Point)
                file_Open = open("data.csv", "a")
                data = str(straight_Point) + "; \n"
                file_Open.write(data)
                file_Open.close()
                RightMotor.forward(0.30)
                LeftMotor.forward(0.30)
                
    except KeyboardInterrupt:
        lidar.stop()
        lidar.disconnect()
    except Exception:
        lidar.stop()
        goback()
    
def goback():
    lidar = RPLidar(None, PORT_NAME)
    main()
    
if __name__ == "__main__":
    main()
