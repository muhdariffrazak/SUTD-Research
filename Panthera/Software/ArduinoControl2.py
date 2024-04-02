import serial
import time

startMarker = '*'
endMarker = '*'
dataStarted = False
dataBuf = ""
messageComplete = False
dataCount = 0

#========================
#========================
    # the functions

def setupSerial(baudRate, serialPortName):
    
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

#========================

def sendToArduino(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('ascii')) # encode needed for Python3


#==================

def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete, dataCount

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("ascii") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker and dataCount < 10:
                dataBuf = dataBuf + x
                dataCount += 1
            else:
                dataBuf = dataBuf + x
                dataStarted = False
                messageComplete = True
                
        elif x == startMarker:
            dataBuf = ''
            dataBuf = dataBuf + x
            dataCount += 1
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

#==================

def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    
    print("Waiting for Arduino to reset")
     
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)



#====================
#====================
    # the program


setupSerial(115200, "COM4")
count = 0
prevTime = time.time()
while True:
            # check for a reply
    arduinoReply = recvLikeArduino()
    if not (arduinoReply == 'XXX'):
        print ("Time %s  Reply %s" %(time.time(), arduinoReply))
        
        # send a message at intervals
    if time.time() - prevTime > 1.0:
        
        if count == 0:
            sendToArduino("100100000")
            count = 1
            prevTime = time.time()
        elif count == 4:
            count = 0
        else:
            count += 1
            
        
            