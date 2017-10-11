from Crypto import Random
from Crypto.Cipher import AES
# ==================================================
# To be added after integration with prediction.py
# from prediction import results
# ==================================================

import base64
import socket
import sys
# ==================================================
# To be removed after integration with prediction.py
import random
import time
# ==================================================

class client:
	def __init__(self, ip_addr, port_num):
		# Create TCP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# Connect to server
		server_address = (ip_addr, port_num)
		print('Initiating connection to %s port %s' % server_address, file=sys.stderr)
		self.sock.connect(server_address)
		print('Connected to %s port %s' % server_address, file=sys.stderr)
			
		# Obtain secret key from local key file
		# TODO: Encode secret_key in key file then perform decode operations
		with open('key') as key:
			secret_key = key.read()
			key.closed
		
		# ===================================================
		# Uncomment this section for Raspberry Pi integration
		# Obtain secret key from thumbdrive in Raspberry Pi
		'''
		secret_key = ' '
		for root, dirs, files in os.walk('/media/pi'):
			if 'key' in files:
				if os.access(join(root, 'key'), os.R_OK):
					with open(join(root, 'key')) as key:
						secret_key = key.read()
						key.closed
						break
		'''
		# ===================================================

		# List of actions available
		self.actions = ['logout  ', 'wavehands', 'busdriver', 'frontback', 'sidestep', 'jumping',
						'jumpingjack', 'turnclap', 'squatturnclap', 'windowcleaning', 'windowcleaner360']
		action = ''
		cumulativepower_list = []

		# Send data until logout action is recieved
		while action != 0:
			#1. Get action, current and voltage from prediction.py
			# TODO: Capture action, current and voltage from prediction.py
			
			# ====================================================
			# Uncomment this section for Raspberry Pi integration
			# Obtain action, current and voltage from prediction.py
			# action = results
			'''
			# Uncomment this section if performing file reading
			# TODO: Check if file has changed since previous results if not wait until new file exists
			with open('send_server') as send_server:
				predicted_results = send_server.read()
				# TODO: predicted_results split into action, current and voltage
				send_server.closed
			'''	
			# ====================================================
			
			# ===================================================
			# Generates random data for testing purposes
			# Expecting an integer for predicted action
			time.sleep(5)
			action = random.randrange(0, 10)
			current = random.uniform(0, 3)
			voltage = random.uniform(0, 5)
			# ===================================================
			
			#1a. Calculates average power since first reading
			power = voltage * current
			voltage_str = str(round(voltage, 2))
			current_str = str(round(current, 2))
			power_str = str(round(power,2))
			cumulativepower_list.append(power)
			#print("cumulativepower List : ", cumulativepower_list)
			cumulativepower_list_avg = float(sum(cumulativepower_list) / len(cumulativepower_list))
			
			#1b. Assemble message
			msg = b'#' + b'|'.join([self.actions[action].encode(), voltage_str.encode(), current_str.encode(), power_str.encode(), str(round(cumulativepower_list_avg, 2)).encode()]) + b'|'
			print('unencrypted msg: ', msg)

			#2. Encrypt readings
			#2a. Apply padding
			length = 16 - (len(msg) % 16)
			msg += bytes([length])*length
			
			#2b. Apply AES-CBC encryption
			iv = Random.new().read(AES.block_size)
			cipher = AES.new(secret_key.encode(), AES.MODE_CBC, iv)
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
