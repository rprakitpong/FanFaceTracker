import numpy as np
import cv2
import os 
import serial
import tkinter as tk
import threading

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
class GUIcontroller(threading.Thread):
    def __init__(self, machine):
        threading.Thread.__init__(self)
        self.start()
        #store reference to machine
        self.machine = machine

    def end(self):
        self.root.quit()

    def setLocation(self, loc):
        self.locLabelString.set('Location: '+str(locationToDeg(loc)))

    def move(self, angle):
        self.machine.sendCommand(angle)

    def update(self):
        self.setLocation(self.machine.location)

    def run(self):
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.end)

        instructionLabel = tk.Label(self.root, text='Click ESC to end')
        instructionLabel.pack()

        self.locLabelString = tk.StringVar()
        self.setLocation(0)
        locLabel = tk.Label(self.root, textvariable=self.locLabelString)
        locLabel.pack()

        moveLeftBtn = tk.Button(self.root, text='Move left (-) ~10 deg')
        moveLeftBtn.bind("<ButtonRelease-1>", lambda x:self.move(degToLocation(-10))) # becomes 1024 steps = 1/4 rev = 10 deg rack movement
        moveLeftBtn.pack()

        moveRightBtn = tk.Button(self.root, text='Move right (+) ~10 deg')
        moveRightBtn.bind("<ButtonRelease-1>", lambda x:self.move(degToLocation(10)))
        moveRightBtn.pack()

        self.root.mainloop()

class CVcontroller():
    def __init__(self, machine):
        # gui paramter
        self.width = 640
        self.height = 480
        # controls parameter
        self.threshold = 0.05 # face has in center of image, with x percent threshold
        self.loBound = int(self.width / 2 - (self.width * self.threshold) / 2)
        self.hiBound = int(self.width / 2 + (self.width * self.threshold) / 2)
        # set up camera
        self.cap = cv2.VideoCapture(1)                                                   # use usb camera, not laptop camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.width)                                     # set Width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)                                   # set Height
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        # store reference to machine
        self.machine = machine
    
    def update(self):
        # get frame
        ret, frame = self.cap.read()

        # get faces
        img = frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        # recolor frame to highlight threshold
        ball = img[:, self.loBound:self.hiBound]
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        img[:, self.loBound:self.hiBound] = ball

        if len(faces) > 0:
            (x,y,w,h) = faces[0]                                                # only detect one face at a time

            # draw face on frame
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

            # make x is center of face instead of left edge
            x = int(x + w / 2)

            # send new movement command if face is found outside threshold bound, and pinion location is still inside rack 
            if x < self.loBound:
                # get movement
                ratio = (self.loBound - x) / self.loBound                                 # farther from loBound, higher ratio
                movement = -int(ratio * self.machine.maxMovement)
                self.machine.sendCommand(movement)
                    
            if x > self.hiBound:     
                # get movement   
                ratio = (x - self.hiBound) / (self.width - self.hiBound)                       # farther from hiBound, higher ratio
                movement = int(ratio * self.machine.maxMovement)
                self.machine.sendCommand(movement)
            
        # show frame
        cv2.imshow('frame', img)
    
    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()

class Machine():
    def __init__(self):
        # set up serial
        self.serialPort = serial.Serial(port = "COM6", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        # rack and pinion state variables
        self.maxMovement = int(255 / 2)
        self.maxLoLocation = self.maxMovement * -7                                            # experimentally determined
        self.maxHiLocation = self.maxMovement * 7
        self.location = 0
        self.isMoving = False


    def sendCommand(self, mov):
        # params
        #   mov = movement to make, can be negative or positive
        # return 
        #   bool = successfully moved or not
        #   int = new location
        # for sending command to serial port

        # if moving wouldnt get pinion out of rack bounds, then send command
        if (self.location + mov > self.maxLoLocation or self.location + mov < self.maxHiLocation) and not self.isMoving:
            # convert to byte in receiveable format, and send
            toSend = self.maxMovement + mov
            toSend_byte = bytes([toSend])
            self.serialPort.write(toSend_byte)
            # update location
            self.location = self.location + mov
            self.isMoving = True
        
    def update(self):
        # if movement is finished, reset input buffer so new commands can be sent
        if self.serialPort.in_waiting > 0:
            self.serialPort.reset_input_buffer()
            self.isMoving = False

    def end(self):
        self.serialPort.close()


# set path to folder where python file is
path = os.path.dirname(os.path.realpath(__file__))
os.chdir(path)

# set up machine
machine = Machine()
# set up tkinter app
app = GUIcontroller(machine)
# set up CVController
cv = CVcontroller(machine)

while(True):
    
    # update CV
    cv.update()
    # update machine
    machine.update()
    # update gui
    app.update()

    # get out of loop, press 'ESC' to quit
    k = cv2.waitKey(30) & 0xff
    if k == 27: 
        break

# end camera
cv.end()
# end machine communication
machine.end()
# end tkinter window
app.end()
