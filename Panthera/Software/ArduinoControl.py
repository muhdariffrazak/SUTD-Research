import serial
import time
from time import sleep
from threading import Thread

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)


def write_n_read():
    while True:
        userInput = input('Get data point?')
        
        if userInput == 'y':
            x = '*100100000*'
            arduino.write(bytes(x, 'ascii'))
            time.sleep(0.05)
            data = arduino.readline().decode('ascii').strip('\r\n')
            print(data)

def read_data():
    while True:
        data = arduino.readlines(4).decode('ascii').split('\r\n').strip('\r\n')
        print(data)
        time.sleep(3)


threadlist = []

threadlist.append(Thread(target=write_n_read()))
threadlist.append(Thread(target=read_data()))

while True:
    for t in threadlist:
        t.start()

    for t in threadlist:
        t.join()
    
  

    
    
