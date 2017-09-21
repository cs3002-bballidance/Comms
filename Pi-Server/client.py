from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes

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
	#key = get_random_bytes(16)
	#print('generating key...')
	#cipher = AES.new(key, AES.MODE_CBC)
	#print('generating cipher...')
	
	# Decrypt data upon receiving
	print('recieving data...')
	secret_key = self.recv(16)
	print('received data:', secret_key)
	
	#iv = Random.new().read(AES.block_size)
	#cipher = AES.new(secret_key,AES.MODE_CBC,iv)
	#encoded = base64.b64encode(iv + cipher.encrypt(msg))
	
	#decryptor = AES.new(key, mode, IV=IV)
	#plain = decryptor.decrypt(ciphertext)
	#print('received data:', plain)
	
	# Encrypt data before sending
	print('sending data...')
	data = (b'logout  ')
	#data = cipher.encrypt(data)
	self.send(data)
	print('sent data:', data)
	
	# Close the connection
	self.close()
	
	# AES encryption of files in Python with PyCrypto
	# http://eli.thegreenplace.net/2010/06/25/aes-encryption-of-files-in-python-with-pycrypto/
	def encrypt_file(key, in_filename, out_filename=None, chunksize=64*1024):
		if not out_filename:
			out_filename = in_filename + '.enc'

		iv = ''.join(chr(random.randint(0, 0xFF)) for i in range(16))
		encryptor = AES.new(key, AES.MODE_CBC, iv)
		filesize = os.path.getsize(in_filename)

		with open(in_filename, 'rb') as infile:
			with open(out_filename, 'wb') as outfile:
				outfile.write(struct.pack('<Q', filesize))
				outfile.write(iv)

				while True:
					chunk = infile.read(chunksize)
					if len(chunk) == 0:
						break
					elif len(chunk) % 16 != 0:
						chunk += ' ' * (16 - len(chunk) % 16)

					outfile.write(encryptor.encrypt(chunk))
	
	def decrypt_file(key, in_filename, out_filename=None, chunksize=24*1024):
		if not out_filename:
			out_filename = os.path.splitext(in_filename)[0]

		with open(in_filename, 'rb') as infile:
			origsize = struct.unpack('<Q', infile.read(struct.calcsize('Q')))[0]
			iv = infile.read(16)
			decryptor = AES.new(key, AES.MODE_CBC, iv)

			with open(out_filename, 'wb') as outfile:
				while True:
					chunk = infile.read(chunksize)
					if len(chunk) == 0:
						break
					outfile.write(decryptor.decrypt(chunk))

				outfile.truncate(origsize)
	
if len(sys.argv) != 3:
	print('Invalid number of arguments')
	print('python client.py [IP address] [Port]')
	sys.exit()
