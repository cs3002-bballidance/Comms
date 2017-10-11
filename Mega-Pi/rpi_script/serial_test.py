import serial
import time
import numpy

#constants
HANDSHAKE_PKT = bytes.fromhex("DD1C")
ACK_PKT = bytes.fromhex("DDCC")
ERR_PKT = bytes.fromhex("DDFD")

ser = serial.Serial('/dev/ttyAMA0',9600)
ser.flushInput()
#ser.open()

#initialize communications
hasReplied = False
while(not hasReplied):
    #1. send a handshake
    ser.write(HANDSHAKE_PKT)
    print("Debug: Sending handshake")
    #2. wait for input then check
    time.sleep(1)
    print("Debug: Waiting for acknowledgement")
    bytesToRead = ser.inWaiting()
    response = ser.read(bytesToRead)
    #3. send an ACK if right
    if response == ACK_PKT:
        print("Debug: Acknowledgement received")
        print()
        hasReplied = True
        ser.write(ACK_PKT)
    else:
        time.sleep(1)
        
#wait for data
while True:
    bytesToRead = ser.inWaiting()
    if (bytesToRead == 14) : #wait until the entire packet arrives
        data = ser.read(bytesToRead)
        print(str(data))
        #next step: save to file
        #data_array = numpy.asarray(data)
        #print(str(data_array))
        #numpy.savetxt("mega_data.csv", data_array, delimiter=",")
        
#ser.close()
