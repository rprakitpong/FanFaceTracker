import numpy as np
import cv2
import os 
import serial
import tkinter as tk
import threading

# rack and pinion state variables
maxMovement = int(255 / 2)
maxLoLocation = maxMovement * -7                                            # experimentally determined
maxHiLocation = maxMovement * 7
location = 0
isMoving = False
# gui paramter
width = 640
height = 480
# controls parameter
threshold = 0.05 # face has in center of image, with x percent threshold
loBound = int(width / 2 - (width * threshold) / 2)
hiBound = int(width / 2 + (width * threshold) / 2)

# for sending command to serial port
def sendCommand(mov):
    # params
    #   mov = movement to make, can be negative or positive
    # return 
    #   bool = successfully moved or not
    #   int = new location

    # if moving wouldnt get pinion out of rack bounds, then send command
    global location
    global isMoving
    if (location + mov > maxLoLocation or location + mov < maxHiLocation) and not isMoving:
        # convert to byte in receiveable format, and send
        toSend = maxMovement + mov
        toSend_byte = bytes([toSend])
        serialPort.write(toSend_byte)
        # update location
        location = location + mov
        app.setLocation(location)
        isMoving = True

def locationToDeg(loc):
    # params
    #   loc = arbitrary unit location/movement used throughout this code
    # return
    #   deg = degree turned
    # MCU get number of steps from loc*10
    # steps/(4096 = steps in full rev) = pinion revolutions turned
    # 12/110 = pinion to rack ratio
    # 360 deg/rev
    return round((loc*10/4096)*(12/110)*360, 5)

def degToLocation(deg):
    # params
    #   deg = degree turned 
    # return
    #   loc = arbitrary unit location/movement
    return int(deg*(4096/10)*(110/12)*(1/360))

# tkinter gui class
class App(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.start()

    def callback(self):
        self.root.quit()

    def setLocation(self, loc):
        self.locLabelString.set('Location: '+str(locationToDeg(loc)))

    def move(self, angle):
        sendCommand(angle)

    def run(self):
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        instructionLabel = tk.Label(self.root, text='Click ESC to end')
        instructionLabel.pack()

        self.locLabelString = tk.StringVar()
        self.setLocation(0)
        locLabel = tk.Label(self.root, textvariable=self.locLabelString)
        locLabel.pack()

        moveLeftBtn = tk.Button(self.root, text='Move left ~10 deg')
        moveLeftBtn.bind("<ButtonRelease-1>", lambda x:self.move(degToLocation(-10))) # becomes 1024 steps = 1/4 rev = 10 deg rack movement
        moveLeftBtn.pack()

        moveRightBtn = tk.Button(self.root, text='Move right ~10 deg')
        moveRightBtn.bind("<ButtonRelease-1>", lambda x:self.move(degToLocation(10)))
        moveRightBtn.pack()

        self.root.mainloop()

# set path to folder where python file is
path = os.path.dirname(os.path.realpath(__file__))
os.chdir(path)

# set up camera
cap = cv2.VideoCapture(1)                                                   # use usb camera, not laptop camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)                                     # set Width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)                                   # set Height
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# set up serial
serialPort = serial.Serial(port = "COM6", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

# set up tkinter app
app = App()

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
        (x,y,w,h) = faces[0]                                                # only detect one face at a time

        # draw face on frame
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

        # make x is center of face instead of left edge
        x = int(x + w / 2)

        # send new movement command if face is found outside threshold bound, and pinion location is still inside rack 
        if x < loBound:
            # get movement
            ratio = (loBound - x) / loBound                                 # farther from loBound, higher ratio
            movement = -int(ratio * maxMovement)
            sendCommand(movement)
                
        if x > hiBound:     
            # get movement   
            ratio = (x - hiBound) / (width - hiBound)                       # farther from hiBound, higher ratio
            movement = int(ratio * maxMovement)
            sendCommand(movement)
        
    # show frame
    cv2.imshow('frame', img)

    # if movement is finished, reset input buffer so new commands can be sent
    if serialPort.in_waiting > 0:
        serialPort.reset_input_buffer()
        isMoving = False

    # get out of loop, press 'ESC' to quit
    k = cv2.waitKey(30) & 0xff
    if k == 27: 
        break

# end camera
cap.release()
cv2.destroyAllWindows()

# end serial
serialPort.close()

# end tkinter window
app.callback()
