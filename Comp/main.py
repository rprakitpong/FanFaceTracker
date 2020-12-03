import numpy as np
import cv2
import os 
import serial

# set path to folder where python file is
path = os.path.dirname(os.path.realpath(__file__))
os.chdir(path)

# set up camera
width = 640
height = 480
cap = cv2.VideoCapture(1) # use usb camera, not laptop camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width) # set Width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height) # set Height
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# set up serial
serialPort = serial.Serial(port = "COM6", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
maxMovement = int(255 / 2)

# set up parameters
threshold = 0.05 # face has in center of image, with x percent threshold
loBound = int(width / 2 - (width * threshold) / 2)
hiBound = int(width / 2 + (width * threshold) / 2)

while(True):
    # get frame
    ret, frame = cap.read()

    # get faces
    img = frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    # recolor frame to highlight threshold
    ball = img[:, loBound:hiBound]
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    img[:, loBound:hiBound] = ball

    if len(faces) > 0:
        (x,y,w,h) = faces[0] # only detect one face at a time
        #print(str(x) + ',' + str(y))

        # draw face on frame
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

        # make x is center of face instead of left edge
        x = int(x + w / 2)

        # send new movement command if face is found outside bound
        if x < loBound and serialPort.in_waiting == 0:
            ratio = (loBound - x) / loBound             # farther from loBound, higher ratio
            movement = int(ratio * maxMovement)
            toSend = maxMovement - movement
            toSend_byte = bytes([toSend])
            serialPort.write(toSend_byte)
        if x > hiBound and serialPort.in_waiting == 0:        
            ratio = (x - hiBound) / (width - hiBound)   # farther from hiBound, higher ratio
            movement = int(ratio * maxMovement)
            toSend = maxMovement + movement
            toSend_byte = bytes([toSend])
            serialPort.write(toSend_byte)

        # if movement is finished, reset input buffer so new commands can be sent
        if serialPort.in_waiting > 0:
            serialPort.reset_input_buffer()
        
    # show frame
    cv2.imshow('frame', img)

    # get out of loop
    k = cv2.waitKey(30) & 0xff
    if k == 27: # press 'ESC' to quit
        break

# end camera
cap.release()
cv2.destroyAllWindows()

# end serial
serialPort.close()