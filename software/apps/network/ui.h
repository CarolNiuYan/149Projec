#ifndef UI_H
#define UI_H

//#include <curses.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/**
 * Initialize the home scene (welcome screen), programs should call
 * this at the beginning if the user initialize a game room.
 */
void init_home(int port_num);

/**
 * Run in a thread to process user input.
 */
void* read_input(void* stat);

/**
 * Run in a thread to move the worm around on the board
 */
void end_control(int winner);

void try_this(int word);

#endif
