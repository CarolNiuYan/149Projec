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

#include "network.h"
//#include "gpu.h"
#include "ui.h"


/**
 * Initialize the home scene (welcome screen), programs should call
 * this at the beginning if the user initialize a game room.
 */
void init_home(int port_num){
  // Print port num message
  printf("You opened control room at port:");

  // Print port num
  char number[10];
  sprintf(number, "%d", port_num);
  printf("%s", number);

  
  // Print waiting message
  //move(++start_row, 0);
  printf("Waiting for connection. . . ");

  // Refresh the display
  //refresh();
}


/**
 * Run in a thread to process user input.
 */
void* read_input(void* stat){

}


/**
 * Show a game over message & winner information, and wait for a key press.
 */
void end_game(int winner) {
  //erase();
  //refresh();
  //timeout(-1);
}
