import serial
import time
import numpy
import csv
import sys
import struct

#constants
HANDSHAKE_PKT = bytes.fromhex("DD1C")
ACK_PKT = bytes.fromhex("DDCC")
ERR_PKT = bytes.fromhex("DDFD")

#instantiate serial
ser = serial.Serial('COM3',9600)
ser.flushInput()

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
        time.sleep(1)
        print("Debug: Waiting for handshake")
        bytesToRead = ser.inWaiting()
        response = ser.read(bytesToRead)
        #3. send an ACK if right
        if response == ACK_PKT:
            print("Debug: Handshake received")
            print()
            hasReplied = True
            ser.write(ACK_PKT)
        else:
            time.sleep(1)
    
    #start timer
    startTime = time.time()
    endTime = time.time()
    
    #wait for data
    while True: #(endTime - startTime) < 1:
         
        #1. wait until the entire packet arrives
        if (ser.inWaiting() >= 25) :
            
            #TODO: checksum after integration
            packet_type = bytearray(ser.read(2))
            checksum = bytearray(ser.read(1))
            
            #2. read data and convert to appropriate values
            #>h big endian, signed int (2 bytes)
            #<B big endian, unsigned int (1 byte)
            (acc1x,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc1y,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc1z,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc2x,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc2y,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc2z,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc3x,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc3y,) = struct.unpack(">h", bytearray((ser.read(2))))
            (acc3z,) = struct.unpack(">h", bytearray((ser.read(2))))
            (curr,) = struct.unpack(">h", bytearray((ser.read(2))))
            (volt,) = struct.unpack(">h", bytearray((ser.read(2))))
            
            writer.writerow({'acc1x': acc1x,
                             'acc1y': acc1y, 
                             'acc1z': acc1z,
                             'acc2x': acc2x,
                             'acc2y': acc2y,
                             'acc2z': acc2z,
                             'acc3x': acc3x,
                             'acc3y': acc3y,
                             'acc3z': acc3z,
                             'curr':  curr,
                             'volt':  volt
                             })
            print('acc1: ', acc1x, acc1y, acc1z)
            print('acc2: ', acc2x, acc2y, acc2z)
            print('acc3: ', acc3x, acc3y, acc3z)
            print('pow: ', curr, volt)
        #3. update timer
        #endTime = time.time()
        
#All done
ser.close()
sys.exit()
