from Crypto import Random
from Crypto.Cipher import AES

import base64
import random
import socket
import sys

class client:
    def __init__(self, ip_addr, port_num):
        
        #Create TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
        #Connect to server
        server_address = (ip_addr, port_num)
        print('Initiating connection to %s port %s' % server_address, file=sys.stderr)
        self.sock.connect(server_address)
        print('Connected to %s port %s' % server_address, file=sys.stderr)
        
        # Setup encryption
        secret_key = b'69p2E2EDdEZP0NDG'
        
        #Vars for simulation -> Remove later on
        self.actions = ['busdriver', 'frontback', 'jumping', 'jumpingjack', 'sidestep',
                        'squatturnclap', 'turnclap', 'wavehands', 'windowcleaner360', 'windowcleaning']
        action = ''
        
        while action != 'logout  ' : #Send data until log-out pose
            #1. Get action from machine learning script -> SIMULATED
                action = input('Enter action (use \'logout  \' to exit): ');
                voltage = b'9999'
                current = b'9999'
                power = b'9999'
                cumulativepower = b'9999'
                msg = b'#' + b'|'.join([action.encode(), voltage, current, power, cumulativepower]) + b'|'
                
                print('unencrypted msg: ', msg)
                
            #2. Encrypt readings
                
                #2a. Apply padding
                length = 16 - (len(msg) % 16)
                msg += bytes([length])*length
                
                iv = Random.new().read(AES.block_size)
                cipher = AES.new(secret_key, AES.MODE_CBC, iv)
                encodedMsg = base64.b64encode(iv + cipher.encrypt(msg))
                print('encrypted msg:   ', encodedMsg)
                
            #3. Send data packet over
                print('sending msg')
                self.sock.sendall(encodedMsg)
                            
        #4. All done, logout.
        self.sock.close()
        sys.exit()
        
if len(sys.argv) != 3:
	print('Invalid number of arguments')
	print('python client.py [IP address] [Port]')
	sys.exit()

ip_addr = sys.argv[1]
port_num = int(sys.argv[2])

my_client = client(ip_addr, port_num)
