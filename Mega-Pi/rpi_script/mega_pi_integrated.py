import RPi.GPIO as GPIO
import serial
import time
import numpy
import csv
import sys
import struct
import logging

#constants
HANDSHAKE_PKT = bytes.fromhex("DD1C")
ACK_PKT = bytes.fromhex("DDCC")
ERR_PKT = bytes.fromhex("DDFD")
RESET_PIN = 17
LOG_FILENAME = 'mega_pi.log'

logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',filename=LOG_FILENAME,level=logging.INFO)
logger.setLevel(logging.DEBUG)
logger.info('Starting {}'.format(__file__))

#instantiate GPIO
logger.info('Initializing GPIO')
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RESET_PIN,GPIO.OUT)
GPIO.output(RESET_PIN,GPIO.HIGH) # pull low then back to high to reset arduino

#instantiate serial
logger.info('Initializing serial interface')
ser = serial.Serial('/dev/ttyAMA0',57600)
GPIO.output(RESET_PIN,GPIO.LOW)
ser.flushInput() # flush any existing serial buffer
logger.info('Resetting arduino before resuming')
time.sleep(2) # sleep for 2 second before pulling the pin back to high
GPIO.output(RESET_PIN,GPIO.HIGH)

with open('mega_data.csv', 'w') as csvfile:
    fieldnames = ['acc1x', 'acc1y', 'acc1z', 'acc2x', 'acc2y', 'acc2z', 'acc3x', 'acc3y', 'acc3z', 'curr', 'volt']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
        
    #initialize communications
    hasReplied = False
    while(not hasReplied):
        #1. send a handshake
        logger.info('Sending handshake to arduino: {}'.format(HANDSHAKE_PKT))
        ser.write(HANDSHAKE_PKT)
        #2. wait for input then check
        time.sleep(1)
        logger.info('Waiting for acknowledgement from arduino')
        bytesToRead = ser.inWaiting()
        response = ser.read(bytesToRead)
        #3. send an ACK if right
        if response == ACK_PKT:
            logger.info('Acknowledgement received')
            hasReplied = True
            ser.write(ACK_PKT)
        else:
            time.sleep(1)
    
    #start timer
    startTime = time.time()
    endTime = time.time()
    logger.debug('start timer at: {}'.format(startTime))
    logger.debug('endTime: {}'.format(endTime))

    count = 0
    #wait for data
    while (endTime - startTime) < 1: #True :
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
            #print('acc1: ', acc1x, acc1y, acc1z)
            #print('acc2: ', acc2x, acc2y, acc2z)
            #print('acc3: ', acc3x, acc3y, acc3z)
            #print('pow: ', curr, volt)
            count = count + 1
            logger.debug('count: {}'.format(count))
        #3. update timer
        endTime = time.time()
    logger.debug('data collected: {}'.format(count))

#All done
ser.close()
logger.info('Exiting {}'.format(__file__))
sys.exit()
