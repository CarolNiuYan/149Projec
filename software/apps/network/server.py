import socket
import sys
import select
import time
import broadcaster

HOST = '0.0.0.0'
PORT = 53535

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('', PORT))

server_socket.listen(5)
print("\n Listning on port: " + str(PORT))

client_socket, (client_ip, client_port) = server_socket.accept()
print("\n Client" + client_ip + "connected successfully\n")

while True:
	in_payload = client_socket.recv(128)
	in_payload = in_payload.decode()
	# deserialize
	payload = in_payload.split()
	print(payload)
	time.sleep(0.100)
client_socket.close()
