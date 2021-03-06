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
DURATION = 10

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
time.sleep(1) # sleep for 1 second before pulling the pin back to high
GPIO.output(RESET_PIN,GPIO.HIGH)

with open('mega_data.csv', 'w') as csvfile:
    fieldnames = ['acc1x', 'acc1y', 'acc1z', 'acc2x', 'acc2y', 'acc2z', 'acc3x', 'acc3y', 'acc3z', 'curr', 'volt']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
        
    #initialize communications
    hasReplied = False
    while(not hasReplied):
        #1. send a handshake
        logger.info('Sending handshake to arduino')
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

    count = 0
    #wait for data
    while (endTime - startTime) < DURATION: #True :
        #1. wait until the entire packet arrives
        if (ser.inWaiting() >= 26) :
            packet_type = bytearray(ser.read(2))
            (checksum,) = struct.unpack(">h", bytearray(ser.read(2)))
            
            #2. read data and convert to appropriate values
            #>h big endian, signed int (2 bytes)
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
            
            calcChecksum = acc1x ^ acc1y ^ acc1z ^ acc2x ^ acc2y ^ acc2z ^ acc3x ^ acc3y ^ acc3z ^ curr ^ volt
            
            if (checksum == calcChecksum) :
                ser.write(ACK_PKT)
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
            else: 
                logger.warn('Packet error')
                ser.write(ERR_PKT)
            count = count + 1
        #3. update timer
        endTime = time.time()
    logger.debug('startTime: {}'.format(startTime))
    logger.debug('endTime: {}'.format(endTime))
    logger.debug('duration: {} secs'.format(int(endTime - startTime)))
    logger.debug('data collected: {}'.format(count))

#All done
ser.close()
logger.info('Exiting {}'.format(__file__))
sys.exit()
