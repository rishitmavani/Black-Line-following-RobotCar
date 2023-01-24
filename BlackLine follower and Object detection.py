import cv2
from gpiozero import Robot, Motor
import numpy as np
import os
from math import cos, sin, pi, floor
from adafruit_rplidar import RPLidar

os.putenv('SDL_FBDEV', '/dev/fb1')
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

RightMotor = Motor(17,22,18)
LeftMotor = Motor(4,24,19)

video_capture = cv2.VideoCapture(-1)
video_capture.set(3, 320)
video_capture.set(4, 240)

scan_data = [0]*360

def main():
    def process_data(lidar_output):
        front=scan_data[180]
        print("front_Process:" + str(front))
        if front < 600:
            LeftMotor.forward(0)
            RightMotor.forward(0)
        else:
            ret, frame = video_capture.read()      
            ret = video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,320)
            ret = video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

            # Convert to grayscale
            crop_img = frame[140:-1, 20:310]
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

            # Gaussian blur
            blur = cv2.GaussianBlur(gray,(5,5),0)

            # Color thresholding
            ret,thresh1 = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)      

            # Erode and dilate to remove accidental line detections
            mask = cv2.erode(thresh1, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find the contours of the frame
            contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
 
            # Find the biggest contour (if detected)
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
                cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
                cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)

                print("x: " + str(cx))
                print("y: " + str(cy))
                
                if cx < (crop_img.shape[1] / 3):
                    print("Turn Left")
                    LeftMotor.forward(0.20)
                    RightMotor.forward(0.35)
                
                elif cx > (crop_img.shape[1] * 2 / 3):
                    print("Turn Right")
                    RightMotor.forward(0.20)
                    LeftMotor.forward(0.35)
                else:
                    print("On Track!")
                    RightMotor.forward(0.30)
                    LeftMotor.forward(0.30)
         
    while True:
        try:
            for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    scan_data[min([359, floor(angle)])] = distance
                front=scan_data[180]
                
                if front < 600:
                    print("front:" + str(front))
                    LeftMotor.forward(0)
                    RightMotor.forward(0)
                    
                else:
                    process_data(scan_data)
        except Exception:
            lidar.stop()
            goback()
        finally:
            lidar.stop()
            lidar.disconnect()
        
def goback():
    lidar = RPLidar(None, PORT_NAME)
    main()
    
if __name__ == "__main__":
    main()
