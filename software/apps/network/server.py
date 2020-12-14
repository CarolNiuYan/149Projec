import socket
import sys
import select
import time
import broadcaster as bc

HOST = '0.0.0.0'
PORT = 53599

print("Setup BLE adv....")
bc.stop_broadcasting()
bc.set_mac_address()
bc.set_broadcast_rate()
bc.set_broadcast_advdata(['ff'] * 24)
bc.start_broadcasting()
print("BLE adv finished")

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('', PORT))

server_socket.listen(5)
print("\n Listning on port: " + str(PORT))

client_socket, (client_ip, client_port) = server_socket.accept()
print("\n Client" + client_ip + "connected successfully\n")
quit_accu = 0

while True:
	in_payload = client_socket.recv(104)
	in_payload = in_payload.decode()
	# deserialize
	payload = in_payload.split()
	print(payload)
	if len(payload) != 24:
		if quit_accu > 5:
			break
		print("Wrong length, skip %d" % quit_accu)
		quit_accu += 1
		time.sleep(0.100)
		continue

	bc.set_broadcast_advdata(payload)
	time.sleep(0.100)

print("Now Exit")
bc.stop_broadcasting()
client_socket.close()
