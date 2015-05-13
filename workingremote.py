?
#!/usr/bin/python
# -*- coding: utf-8 -*-
 
from Tkinter import Tk, W, E, S, N
from ttk import Frame, Button, Label, Style
from ttk import Entry
 
import serial
from random import randint
 
 
 
from Tkinter import *
 
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
 
import time
from numpy import arange, sin, pi
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
 
#_______________________________________________________________________________________
#_______________________________________ classes w
#_______________________________________________________________________________________
 
 
#_______________________________________________________________________________________
#_______________________________________ variables _____________________________________
#_______________________________________________________________________________________
 
 
AUTONOMOUS_MODE = 1
REMOTE_CONTROL_MODE = 2
 
# serial.Serial(port, bps, bytesize, parity bit, stopp bit, timeout)
ser = serial.Serial('COM34',115200, 8, serial.PARITY_NONE, 1,0) # ser is the variable for the serial port, i.e. bluetooth
 
# the variable 'mode' is defined in the control frame, doesnt work otherwise!
 
#_______________________________________________________________________________________
#_______________________________________ functions _______________________________________
#_______________________________________________________________________________________
def quit(ser):
    ser.close()
    root.destroy()
 
     
# root frame:
root = Tk()
frame = Frame(root)
frame.grid()
 
bottomFrame = Frame(root,height=100, width=100)
bottomFrame.grid(row=2, column=1, columnspan=2)
 
 
 
 
 
# __________________________ Functions for Autonomous mode ______________________________
u = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
 
 
def BluetoothUpdate():
    data = ser.read(1)
    print data
    '''
    if data != "":
        if ( ord(data[1]) == 255):
            for index in range(len(data)):
                u.append(ord(data[index]))
                del u[0]
                print ord(data[index])
                print ord(data)
    root.after(10, BluetoothUpdate)
        '''
    '''
    if  tem:
        print "walla"
        typ = temp
        root.after(1000, BluetoothUpdate)
    temp = ser.read(1)
     
    if  temp:
        val = temp
        print 'n'
        if hex(ord(typ)) == hex(0x10):
            print("Distance from the right wall, rear sensor:")
        elif hex(ord(typ)) == hex(0x11):
            print("Distance from the right wall, front sensor:")
        elif hex(ord(typ)) == hex(0x12):
            print("Distance from the front sensor:")
        elif hex(ord(typ)) == hex(0x13):
            print("Distance from the left wall, front sensor:")    
        elif hex(ord(typ)) == hex(0x14):
            print("Distance from the left wall, rear sensor:")
        elif hex(ord(typ)) == hex(0x15):
            print("Distance driven:")
        elif hex(ord(typ)) == hex(0x16):
            print("Tape sensor floor:")
        elif hex(ord(typ)) == hex(0x17):
            print("Angular speed:")
        elif hex(ord(typ)) == hex(0x18):
            print("Angle to the right wall")    
        elif hex(ord(typ)) == hex(0x19):
            print("Angle to the left wall")    
        print "Value:", hex(ord(val))
    root.after(1000, BluetoothUpdate)
 
   '''
def forward(): 
    print "forward!"
    stringToWrite = "A"
    print stringToWrite
    ser.write(stringToWrite)
     
         
def forwardLeft():
    print "forward left!"
    stringToWrite = "B"
    print stringToWrite
    ser.write(stringToWrite)    
     
def forwardRight():
    print "forward right!"
    stringToWrite = "C"
    print stringToWrite
    ser.write(stringToWrite)    
  
def rotateLeft():
    print "rotate left!"
    stringToWrite = "D"
    print stringToWrite
    ser.write(stringToWrite)    
     
         
def rotateRight():
    print "rotate right!"
    stringToWrite = "E"
    print stringToWrite
    ser.write(stringToWrite)    
         
def backwards():
    print "backwards!"
    stringToWrite = "F"
    print stringToWrite
    ser.write(stringToWrite)
 
 
         
def stopp():
    print "Stopp!"
    stringToWrite = "G"
    print stringToWrite
    ser.write(stringToWrite)   
 
         
def onKeyPress(event):
    if event.char == "w":
        forward()
    elif event.char == "q":
        forwardLeft()
    elif event.char == "e":
        forwardRight()
    elif event.char == "a":
        rotateLeft()
    elif event.char == "d":
        rotateRight()
    elif event.char == "s":
        backwards()
    elif event.char == "o":
        quit(ser)
 
def onKeyPressSpace(event):
    stopp()
 
# _______________________________________ bottomFrame _______________________________________
quitButton = Button(bottomFrame,text="Quit", command=lambda: quit(ser))
quitButton.pack(side=LEFT) 
quitButton.pack(expand=4)
 
 
 
 
# _______________________________________ main loop __________________________________________
#root.after(10, BluetoothUpdate)
 
 
 
root.bind("w",onKeyPress)
root.bind("q",onKeyPress)
root.bind("e",onKeyPress)
root.bind("a",onKeyPress)
root.bind("s",onKeyPress)
root.bind("d",onKeyPress)
root.bind("<space>",onKeyPressSpace)
 
 
root.mainloop()