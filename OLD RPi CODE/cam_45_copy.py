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


def socketListener():

    start_stop = 38

    GPIO.setwarnings(False)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(start_stop, GPIO.OUT)

    while True:
        msgString = input()
        if(msgString == 'STOP'):
           print('Stopping');
           GPIO.output(start_stop, GPIO.LOW)
        if (msgString == 'START'):
           print('Started')
           GPIO.output(start_stop, GPIO.HIGH)

    s.close()

socList = Thread(target=socketListener, args=())

socList.start()


exit(0)
