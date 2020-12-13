#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "socket.h"
#include "ui.h"

// Open a server socket and return the port number and server socket fd
static void open_control_room(unsigned short *port, int *server_socket_fd){
  // Open a server socket
  *server_socket_fd = server_socket_open(port);
  if (*server_socket_fd == -1) {
    perror("Server socket was not opened");
    exit(2);
  }
  // Start listening for connections
  if (listen(*server_socket_fd, 1)) {
    perror("listen failed");
    exit(2);
  }
}

// connect to an existing control room
// return the opponent socket fd
static void join_control_room(char *peer_hostname, unsigned short peer_port, int *socket_fd) {
  *socket_fd = socket_connect(peer_hostname, peer_port);
  if (*socket_fd == -1) {
    perror("Failed to connect");
    exit(2);
  }

}

// accept connection from new opponent
// return the opponent port
static void accept_connection(int server_socket_fd, int *client_socket_fd) {
  *client_socket_fd = server_socket_accept(server_socket_fd);
  if (*client_socket_fd == -1) {
    perror("Accept failed");
    exit(2);
  }
  return;
}

// receive other's input: TBD
// if fail to get input, set offline to true
static void* get_client_input(int client_socket_fd, float *client_msg) {
  ssize_t rc;
  while(1) {
    float x, y, z;
    rc = read(client_socket_fd, &x, sizeof(float));
    if (rc==-1 || rc==0) {
        // Handle cases that the opponent is offline
        close(client_socket_fd);
        break;
    }
    client_msg[0] = rc;
    rc = read(client_socket_fd, &y, sizeof(float));
    if (rc==-1 || rc==0) {
        // Handle cases that the opponent is offline
        close(client_socket_fd);
        break;
    }
    client_msg[1] = rc;
  }
  return NULL;
}

// send row and col number to opponent
// if we quit, send -1
// if cannot send, we think opponent quitted, return -1
// On success, return 0 
static int send_input(int client_fd, float x, float y) {
  ssize_t rc;
  // Send coordinates of piece to opponent
  rc=write(client_fd,&x,sizeof(float));
  if (rc==-1) {
    close(client_fd);
    return -1;
  }
  rc=write(client_fd,&y,sizeof(float));
  if (rc==-1) {
    close(client_fd);
    return -1;
  }
  return 0;
}

#endif
