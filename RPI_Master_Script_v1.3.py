"""@package RPi_Master_Script

Takes pictures and checks for faces in each picture. If there was a face found
it uploads the picture to thte server and also sends a message to the server
announcing that a file was uploaded and provides the name of said file

Communication via serial from an arduino Mega is providing a constant stream of
information about the movement of the robot and any obsticals in its way.
this data is then processed and instructions are given back to a different
Arduino Due for it to then carry out by updating the motor speeds
    
"""
import cv2
import numpy as np
import picamera
from picamera.array import PiRGBArray
import time
import math
import FTPUpload as ftp
import os
import socket
import sys
import RPi.GPIO as GPIO
from threading import Thread
import serial

#########################################################################
# GLOBAL VARIABLES #

SocketConnected = False

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

motorControl = None
sensorData = None

motorControlConnected = False
sensorDataConnected = False

motorPort = '/dev/ttyACM0'
sensorPort = '/dev/ttyACM1'

ArduinoState = 1
ArduinoPreviousState = 0

oldDX = 0
oldDY = 0

isTakingPictures = True

##########################################################################

def cameraStream():
    """Documentation for the cameraStream thread

    Takes a picture and starts a new faceDetector thread for facial detection.
    Does this repeatedly until the program ends
    
    """
    global isTakingPictures
    camera = picamera.PiCamera()
    camera.vflip = False

    fileCounter = 0
    
    while True:
        while isTakingPictures:
            filename = 'image' + str(fileCounter) + '.jpg'
            faceThread = Thread(target=faceDetector, args=(filename,)) #comma is needed
            camera.capture(filename)
            fileCounter += 1;
            faceThread.start()
            time.sleep(0.1)
        
    camera.close()


def faceDetector(filename):
    """Documentation for the faceDetector thread

    Opens an image file and searches for a face by means of haarcascades. If a face is found it uploads the picture to the server.
    It also changes the state of GPIO start_stop to LOW
    """

    global isTakingPictures
    
    fFace2 = cv2.CascadeClassifier('/home/pi/Downloads/opencv-3.0.0/data/haarcascades_cuda/haarcascade_frontalface_default.xml')
    image = np.asarray(cv2.imread(filename))
    grayScale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    numFaces = len(fFace2.detectMultiScale(grayScale, 1.3, 4))

    if numFaces > 0: #if there is a face
        Stop()
        ftp.Upload(filename)
        print('IMAGE')
        WriteToServer('IMAGE ' + filename)
        isTakingPictures = False
        
    os.remove(filename)

    
def socketListener():
    """Documentation for the socketListener Class

    Connects to the server and listenes for messages,
    then changes the states of the GPIO pins to match the status sent by the server
    as well as sending serial data to the arduino communicating the message to the arduino
    """
    TCP_IP = '192.168.168.1'
    TCP_PORT = 10000
    BUFFER_SIZE = 1024

    global s
    global SocketConnected
    global isTakingPictures
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #assigns the variable s to a generic socket 
    SocketConnected = True
    
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #assigns the variable s to a generic socket
            s.connect((TCP_IP, TCP_PORT)) #connects s to the server at the IP and PORT
            SocketConnected = True
        except Exception as e:
            #print(e)
            SocketConnected = False
        while SocketConnected:
            try:
                msg = s.recv(BUFFER_SIZE) #gets bytes from server and convers to text
                msgString = msg.decode('ASCII')
                print(msg)
                if(len(msgString) > 0):
                    if(msgString == 'STOP'):
                       print('Stopping');
                       Stop()
                    if (msgString == 'START'):
                       print('Started')
                       #DriveForward()
                       DriveCircle()
                    if(msgString == 'KILL'):
                        print('Killing')
                        Stop()
                    if(msgString == 'SAFE'):
                        print('Safe')
                    if(msgString == 'UNSAFE'):
                        print('Unsafe')
                        #Stop()
                    if(msgString == 'MATCHYES'):
                        print('Match')
                        isTakingPictures = False
                        Stop()
                    if(msgString == 'MATCHNO'):
                        print('Not a Match');
                        isTakingPictures = True
                
            except:
                print('Disconected')
                SocketConnected = False
                s.close()
                break

            
def DriveForward():
    global motorControl
    try:
        motorControl.write('120,69\n'.encode('ASCII'))
    except Exception as e:
        None


def DriveBackward():
    global motorControl
    try:
        motorControl.write('60,160\n'.encode('ASCII'))
    except Exception as e:
        None


def Turn(m1,m2):
    global motorControl
    try:
        motorControl.write((str(m1) + ',' + str(m2) + '\n').encode('ASCII'))
    except Exception as e:
        None


def DriveCircle():
    global motorControl
    try:
        motorControl.write('60,69\n'.encode('ASCII'))
    except Exception as e:
        None


def Stop():
    global motorControl
    try:
        motorControl.write('0,0\n'.encode('ASCII'))
    except Exception as e:
        None

        
def WriteToServer( str ):
    global SocketConnected
    global s
    
    if SocketConnected:
        str = str + ' DEBUG '
        s.send(str.encode('ASCII'))
        time.sleep(0.2)

    
def ReadSerial():
    """Documentation for the ReadSerial Thread
    Serial data is read in through /dev/tty/ACM0 which is what the arduino communicates on
    The data is passed to a function for parsing"""

    global sensorData
    global sensorDataConnected

    while True:
        if sensorDataConnected:
            while sensorData is not None:
                sensorData.flushInput() #Flush buffer to avoid backing up data and lagging updates
                mesg = ""
                try:
                    mesg = sensorData.readline().decode('ASCII')
                except Exception as e:
                    None
                finally:
                    HandleSerial(mesg)

        
def HandleSerial(mesg):
    """Documentation for HandleSerial Function

    This function parses each bit of data and sends it to the server with the proper formmating"""
    global sensorData

    #Arduino Message Format: x1,y1,x2,y2,IR,SONIC,VOLTAGE

    mesg = mesg[:-2]  #to remove new line character from end of string
    print(mesg)
        
    data = mesg.split(',') #splits message into each feild
    
    if len(data) == 7: #if the message made it in full - with all 7 fields
        UpdateLocation(data[0], data[1], data[2], data[3], data[4], data[5])
        WriteToServer('VOLTAGE ' + data[6])


def UpdateLocation(x1, y1, x2, y2, ir, sonic):
    """Documentation for UpdateLocation Function

    This function is for calculation of the angle that the robot is facing
    and sending that back to the server"""
    
    global ArduinoState
    global ArduinoPreviousState
    global oldDX
    global oldDY
    dX = 0
    dY = 0

    try:
        dX = int(x1) - oldDX
        dY = int(y1) - oldDY
    except Exception as e:
        None

    theta = math.atan2(dY, dX)

    if(dX == 0 and dY == 0):
        theta = 0
    
    WriteToServer('LOCATION ' + str(dX) + ' ' + str(dY) + ' ' + str(theta))
    oldDX = dX
    oldDY = dY

    #only update the arduino when something changes
    ArduinoPreviousState = ArduinoState
    
    if int(ir) > 600 or float(sonic) < 7:
        ArduinoState = 0 #stop the arduno
    else:
        ArduinoState = 1 #drive the arduino

    if ArduinoPreviousState is not ArduinoState:
        if ArduinoState is 0:
            Stop()
        elif ArduinoState is 1:
            DriveForward()


def setupSerialConnections():
    """ Documentation for setupSerialConnections Thread
    Connects the two arduinos to the correct /dev/ttyACM port and
    updated the boolean variables accordingly
    
    """
    
    global motorControl
    global motorControlConnected
    global motorPort
    global sensorData
    global sensorDataConnected
    global sensorPort

    # /dev/ttyACM[0/1] are the only ports needed

    while True:
        if not motorControlConnected:
            try:
                motorControl = serial.Serial(motorPort, 115200)
                tmpStr = motorControl.readline().decode('ASCII')
                print(tmpStr)
                if tmpStr.startswith('MOTORS'):
                    motorControlConnected = True
                    motorControl.write('CONNECTED'.encode('ASCII'))
                    print('>> Connected to Arduino Due on ' + motorPort + '\n')
                elif tmpStr.startswith('SENSORS'):
                    tmpStr = motorPort
                    motorPort = sensorPort
                    sensorPort = tmpStr
                    print('SWAP 1')
                else:
                    print(tmpStr)
                    
            except Exception as e:
                motorControlConnected = False
                print(e)
        if not sensorDataConnected:
            try:
                sensorData = serial.Serial(sensorPort, 115200)
                tmpStr = sensorData.readline().decode('ASCII')
                print(tmpStr)
                if tmpStr.startswith('SENSORS'):
                    sensorDataConnected = True
                    sensorData.write('CONNECTED'.encode('ASCII'))
                    print('>> Connected to Arduino Mega 2560 on ' + sensorPort + '\n')
                elif tmpStr.startswith('MOTORS'):
                    tmpStr = sensorPort
                    sensorPort = motorPort
                    motorPort = tmpStr
                    print('SWAP 2')
                else:
                    print(tmpStr)
            except Exception as e:
                sensorDataConnected = False
                print(e)
        time.sleep(1)



#####################################################################
#Threads

camStr = Thread(target=cameraStream, args=())
socList = Thread(target=socketListener, args=())
serRead = Thread(target=ReadSerial, args=())
serSetup = Thread(target=setupSerialConnections, args=())

camStr.start()
socList.start()
serSetup.start()
serRead.start()


