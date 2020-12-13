import socket
import sys
import select
import platform
import receiver
import time

HOST = "73.63.163.204"
#HOST = "127.0.0.1"
PORT = 53535

connection_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
connection_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
connection_socket.connect((HOST, PORT))
print("\n Connected to on port: " + str(PORT))

while True:
	payload = receiver.get_adv()
	payload_serialize = " ".join(["{:03d}".format(elem) for elem in payload])
	
	payload_serialize = payload_serialize.encode()
	#print("payload size is %d\n", sys.getsizeof(payload_serialize))
	connection_socket.send(payload_serialize)
	print("Payload delivered\n")
	time.sleep(0.100)

connection_socket.close()