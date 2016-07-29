import picamera
import time
import FTPUpload as ftp

camera = picamera.PiCamera()
camera.vflip = True
end = time.time() + 30
x=0
while time.time() < end:
    filename = 'image' + str(x) + '.jpg'
    camera.capture(filename)
    ftp.Upload(filename)
    x+=1

camera.close()

exit(0)
