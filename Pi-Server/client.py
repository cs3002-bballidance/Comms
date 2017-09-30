from Crypto import Random
from Crypto.Cipher import AES
#from Crypto.Random import get_random_bytes

from binascii import hexlify
import base64
import random
import socket
import sys

class client:
	ip_addr = sys.argv[1]
	port_num = int(sys.argv[2])
	
	# Create a TCP/IP socket
	print('creating socket')
	self = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	
	# Connect the socket to the address of the server
	client_address = (ip_addr, port_num)
	print('connecting to %s port %s' % client_address, file=sys.stderr)
	self.connect((ip_addr, port_num))
	
	# Send and receive data
	secret_key = b'69p2E2EDdEZP0NDG'
	print('key is...', secret_key)
	
	print('generating IV...')
	iv = Random.new().read(AES.block_size)
	#print('generating IV...', hexlify(iv))
	
	print('generating cipher...')
	cipher = AES.new(secret_key, AES.MODE_CBC, iv)
	
	# Decrypt data upon receiving
	print('ready to receive data...')
		
	# Encrypt data before sending
	print('logging off...')
	action = 'logout  '
	voltage = b'9999'
	current = b'9999'
	power = b'9999'
	cumulativepower = b'9999'
	
	msg = b'#' + b'|'.join([action.encode(), voltage, current, power, cumulativepower]) + b'|'
	
	# Apply padding
	length = 16 - (len(msg) % 16)
	msg += bytes([length])*length
	
	print('message is:        ', msg)
	encoded = base64.b64encode(iv + cipher.encrypt(msg))
	print('encrypted message: ', encoded)
	
	data = self.recv(1024)
	while 1:
		print('sending data...')
		self.sendall(encoded)
	
	# Close the connection
	print('closing connection')
	self.close()
	sys.exit()
	
if len(sys.argv) != 3:
	print('Invalid number of arguments')
	print('python client.py [IP address] [Port]')
	sys.exit()