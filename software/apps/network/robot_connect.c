//#include <curses.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>


#include "socket.h"
#include "ui.h"
#include "network.h"

//#include "gpu.h"
//#include "info.h"
//#include "util.h"


bool host;

int main (int argc, char** argv) {
	int rc;
	// Check the argument number is correct
  	if(argc != 1 && argc != 3) {
      fprintf(stderr, "Usage: %s [<host name> <port number>]\n", argv[0]);
      exit(1);
  	}

  	// Declare sockets and port
  	int server_socket_fd;
  	unsigned short server_port;
  	int socket_fd;
  	int client_msg[3];

  	// Check whether we want to connect to other control room
  	if(argc == 3) {
      host=false;

      // Unpack arguments
      char* peer_hostname = argv[1];
      unsigned short peer_port = atoi(argv[2]);

      // Join the host's game
      join_control_room(peer_hostname, peer_port, &socket_fd);
  	} else {
      host=true;

      // Initialize the home game
      open_control_room(&server_port, &server_socket_fd);

      // Initialize the home game display
      init_home(server_port);
      // Accept connection
      accept_connection(server_socket_fd, &socket_fd);
  	}

  	// Transmit signals 
  	if(host) {
  		get_client_input(socket_fd);
  	} else {
  		send_input(socket_fd, 33, 42, 52);
  	}
}
