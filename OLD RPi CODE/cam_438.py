"""@package cam_101

Takes pictures and checks for faces in each picture. If there was a face found it uploads the picture to thte server and also sends a message to the server
announcing that a file was uploaded and also the name of the file
    
"""
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

start_stop = 38  #GPIO start_stop pin
kill = 40; #GPIO pin for killing

GPIO.setwarnings(False)

#sets up GPIO settings
GPIO.setmode(GPIO.BOARD)
GPIO.setup(start_stop, GPIO.OUT)
GPIO.setup(kill, GPIO.OUT)

GPIO.output(start_stop, GPIO.HIGH)

global Connected
Connected = False
global s
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def cameraStream():
    """Documentation for the cameraStream thread

    Takes a picture and starts a new faceDetector thread for facial detection.
    Does this repeatedly until the program ends
    
    """
    
    camera = picamera.PiCamera()
    camera.vflip = True

    fileCounter = 0
    
    while True:
        filename = 'image' + str(fileCounter) + '.jpg'
        faceThread = Thread(target=faceDetector, args=(filename,)) #comma is not a mistake
        camera.capture(filename)
        fileCounter += 1;
        faceThread.start()

    camera.close()


def faceDetector(filename):
    """Documentation for the faceDetector thread

    Opens an image file and searches for a face by means of haarcascades. If a face is found it uploads the picture to the server.
    It also changes the state of GPIO start_stop to LOW
    """
    #fFace1 = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades/haarcascade_frontalface_alt.xml')
    fFace2 = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades_cuda/haarcascade_frontalface_default.xml')
    #pFace = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades_cuda/haarcascade_profileface.xml')
    image = np.asarray(cv2.imread(filename))
    grayScale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    numFaces = len(fFace2.detectMultiScale(grayScale, 1.3, 4))

    if numFaces > 0: #if there is a face
        ftp.Upload(filename)
        print('IMAGE')
        WriteToServer('IMAGE ' + filename)
        GPIO.output(start_stop, GPIO.LOW) #changes state of start_stop to LOW
    os.remove(filename)
    
def socketListener():
    """Documentation for the socketListener Class

    Connects to the server and listenes for messages, then changes the states of the GPIO pins to match the status sent by the server
    """
    TCP_IP = '192.168.168.1'
    TCP_PORT = 10000
    BUFFER_SIZE = 1024

    global s
    global Connected
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #assigns the variable s to a generic socket 
    connected = True
    print('Socket Created')
    
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #assigns the variable s to a generic socket
            s.connect((TCP_IP, TCP_PORT)) #connects s to the server at the IP and PORT
            Connected = True
        except Exception as e:
            #print(e)
            Connected = False
        while Connected:
            try:
                msg = s.recv(BUFFER_SIZE) #gets bytes from server and convers to text
                msgString = msg.decode('ASCII')
                
                if(len(msgString) > 0):
                    if(msgString == 'STOP'):
                       print('Stopping');
                       GPIO.output(start_stop, GPIO.LOW)
                    if (msgString == 'START'):
                       print('Started')
                       GPIO.output(start_stop, GPIO.HIGH)
                    if(msgString == 'KILL'):
                        print('Killing')
                        GPIO.output(kill, GPIO.HIGH)
                    if(msgString == 'SAFE'):
                        print('Safe')
                    if(msgString == 'UNSAFE'):
                        print('Unsafe')
                
            except:
                print('Disconected')
                s.close()
                break

def socketWriter():
    TEST_VOLTAGE = 5
    TEST_LOCATION = 0
    
    while True:
        WriteToServer('VOLTAGE ' + str(TEST_VOLTAGE))
        WriteToServer('LOCATION' + str(TEST_LOCATION))
        time.sleep(1)

def WriteToServer( str ):
    global Connected
    global s
    
    if Connected:
        s.send(str.encode('ASCII'))
    

camStr = Thread(target=cameraStream, args=())
socList = Thread(target=socketListener, args=())
socWrite = Thread(target=socketWriter, args=())

camStr.start()
socList.start()
socWrite.start()


