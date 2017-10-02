import serial
import time
import numpy
import csv
import sys

#constants
HANDSHAKE_PKT = bytes.fromhex("DD1C")
ACK_PKT = bytes.fromhex("DDCC")
ERR_PKT = bytes.fromhex("DDFD")

#instantiate serial
ser = serial.Serial('COM3',9600)
ser.flushInput()
#ser.open()

#TODO: create file

with open('mega_data.csv', 'w') as csvfile:
    fieldnames = ['acc1x', 'acc1y', 'acc1z', 'acc2x', 'acc2y', 'acc2z', 'acc3x', 'acc3y', 'acc3z', 'curr', 'volt']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
        
    #initialize communications
    hasReplied = False
    while(not hasReplied):
        #1. send a handshake
        print("Debug: Sending handshake")
        ser.write(HANDSHAKE_PKT)
        #2. wait for input then check
        print("Debug: Waiting for Acknowledgement")
        time.sleep(1)
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
    
    startTime = time.time()
    endTime = time.time()
    
    #wait for data
    while (endTime - startTime) < 1:
        bytesToRead = ser.inWaiting()
        if (bytesToRead == 25) : #wait until the entire packet arrives
            data = bytearray(ser.read(bytesToRead))
            
            #print(str(rawdata))
            
            #checksum! -> send error if wrong, send ack
            
            #next step: write to file
            data_array = numpy.asarray(data)
            print(str(data_array))
            writer.writerow({'acc1x': data_array[3],
                             'acc1y': data_array[5],
                             'acc1z': data_array[7],
                             'acc2x': data_array[9],
                             'acc2y': data_array[11],
                             'acc2z': data_array[13],
                             'acc3x': data_array[15],
                             'acc3y': data_array[17],
                             'acc3z': data_array[19],
                             'curr':  data_array[21],
                             'volt':  data_array[23]
                             })
        endTime = time.time()
#All done
ser.close()
sys.exit()
