import cv2
import numpy as np
import picamera
from picamera.array import PiRGBArray
import time
import FTPUpload as ftp
import os
import socket
import sys
import RPi.GPIO as GPIO
from threading import Thread

start_stop = 38
kill = 40;

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(start_stop, GPIO.OUT)
GPIO.setup(kill, GPIO.OUT)

GPIO.output(start_stop, GPIO.HIGH)

def cameraStream():
    camera = picamera.PiCamera()
    camera.vflip = True

    fileCounter = 0
    
    while True:
        filename = 'image' + str(fileCounter) + '.jpg'
        faceThread = Thread(target=faceDetector, args=(filename,))
        camera.capture(filename)
        fileCounter += 1;
        faceThread.start()

    camera.close()

def faceDetector(filename):
    #fFace1 = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades/haarcascade_frontalface_alt.xml')
    fFace2 = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades_cuda/haarcascade_frontalface_default.xml')
    #pFace = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades_cuda/haarcascade_profileface.xml')
    image = np.asarray(cv2.imread(filename))
    grayScale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    numFaces = len(fFace2.detectMultiScale(grayScale, 1.75, 4))

    if numFaces > 0:
        print(numFaces)
        ftp.Upload(filename)
        GPIO.output(start_stop, GPIO.LOW)
    os.remove(filename)
    
    

def socketListener():
    TCP_IP = '192.168.168.1'
    TCP_PORT = 10000
    BUFFER_SIZE = 1024

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket Created')

    s.connect((TCP_IP, TCP_PORT))

    while True:
        msg = s.recv(BUFFER_SIZE) #gets bytes from server and convers to text
        msgString = msg.decode('ASCII')
        if(msgString == 'STOP'):
           print('Stopping');
           GPIO.output(start_stop, GPIO.LOW)
        if (msgString == 'START'):
           print('Started')
           GPIO.output(start_stop, GPIO.HIGH)
        if(msgString == 'KILL'):
            print('Killing')
            GPIO.output(kill, GPIO.HIGH)
            #break

    s.close()

camStr = Thread(target=cameraStream, args=())
socList = Thread(target=socketListener, args=())

camStr.start()
socList.start()


exit(0)
