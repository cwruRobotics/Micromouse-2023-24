#include "API.h"
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

// #define DEBUG
// #define SIM

#ifdef DEBUG
  #define log(...) fprintf(stderr, __VA_ARGS__); fflush(stderr)
  #define logf(...) fprintf(stderr, __VA_ARGS__); fflush(stderr)
  #define logln(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); fflush(stderr)
  #define LOGGING 1
#else
  #define log(...)
  #define logf(...)
  #define logln(...)
  #define LOGGING 0
#endif

#define MAZE_SIZE 10
#define MAZE_HEIGHT 10
#define MAZE_WIDTH 10

// Represents cardinal direction with respect to the maze
enum cardinal_t {NORTH, EAST, SOUTH, WEST};
// Used as parameters to the motor functions
enum turning_direction_t { LEFT, RIGHT };

// Single node of maze
typedef struct Node {
  // X-coordinate
  uint8_t x;
  // Y-coordinate
  uint8_t y;
  // Projected distance to goal
  uint8_t dist;
  // Next node to flood
  struct Node *next;
} Node;

// 2D array of maze nodes
Node mazeNodes[MAZE_WIDTH][MAZE_HEIGHT];
// 3D array of maze walls (X, Y, V/H)
bool mazeWalls[MAZE_WIDTH + 1][MAZE_HEIGHT + 1][2] = {false};
// 1D array of nodes representing the coords of the path the robot is currently taking
Node *path[MAZE_SIZE * MAZE_SIZE];
int pathEnd = 0;
// Current robot directional and positional coords
uint8_t facing = NORTH;
uint8_t xPos = 0;
uint8_t yPos = 0;

bool backtracking = false;
bool solving = false;

void floodFill(int goal[][2], int size) {
  #ifdef SIM
    clearAllText();
  #endif
  // Next node to update
  Node *curr;
  // Last node to update
  Node *last;

  // Add goal nodes to queue
  for(int i = 0; i < size; i++) {
    if(i == 0) {
      curr = &mazeNodes[goal[i][0]][goal[i][1]];
    }
    else {
      last->next = &mazeNodes[goal[i][0]][goal[i][1]];
    }
    last = &mazeNodes[goal[i][0]][goal[i][1]];
    mazeNodes[goal[i][0]][goal[i][1]].dist = 0;
  }

  // Add unchecked nodes to queue and update projected distance from goal until all nodes have been updated
  while(curr) {
    #ifdef SIM
      char distance_string[3];
      sprintf(distance_string, "%d", curr->dist);
      setText(curr->x, curr->y, distance_string);
    #endif
    // If north side of node is open, add north node to queue
    if(curr->y + 1 < MAZE_WIDTH && !mazeWalls[curr->x][curr->y + 1][1] && mazeNodes[curr->x][curr->y + 1].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y + 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If east side of node is open, add east node to queue
    if(curr->x + 1 < MAZE_HEIGHT && !mazeWalls[curr->x + 1][curr->y][0] && mazeNodes[curr->x + 1][curr->y].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x + 1][curr->y];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If south side of node is open, add south node to queue
    if(curr->y >= 1 && !mazeWalls[curr->x][curr->y][1] && mazeNodes[curr->x][curr->y - 1].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If west side of node is open, add west node to queue
    if(curr->x >= 1 && !mazeWalls[curr->x][curr->y][0] && mazeNodes[curr->x - 1][curr->y].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x - 1][curr->y];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    curr = curr->next;
  }
}

// Set up maze by assigning starting distances for each node
void initialize() {
  // Iterate through all nodes, setting initial values
  for(int x = 0; x < MAZE_WIDTH; x++) {
    for(int y = 0; y < MAZE_HEIGHT; y++) {
      mazeNodes[x][y].x = x;
      mazeNodes[x][y].y = y;
      mazeNodes[x][y].dist = 100;
      mazeNodes[x][y].next = NULL;
    }
  }
  int goal[4][2] = {{MAZE_WIDTH / 2, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2, MAZE_HEIGHT / 2 - 1}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2 - 1}};
  // Calculate distances from center using flood fill
  floodFill(goal, 4);
}

// Recalculate maze distances for each node
void recalcMaze(int goal[][2], int size) {
  // Iterate through all nodes, resetting distances
  for(int x = 0; x < MAZE_WIDTH; x++) {
    for(int y = 0; y < MAZE_HEIGHT; y++) {
      mazeNodes[x][y].dist = 100;
      mazeNodes[x][y].next = NULL;
    }
  }
  // Recalculate distances from center using flood fill
  floodFill(goal, size);
}

// Update walls from sensor readings
void updateWalls() {
  bool left = wallLeft();
  bool front = wallFront();
  bool right = wallRight();
  switch(facing) {
    // Facing North
    case NORTH:
      mazeWalls[xPos][yPos][0] = left;
      mazeWalls[xPos][yPos + 1][1] = front;
      mazeWalls[xPos + 1][yPos][0] = right;
      #ifdef SIM
        if(left) {
          setWall(xPos, yPos, 'w');
        }
        if(front) {
          setWall(xPos, yPos, 'n');
        }
        if(right) {
          setWall(xPos, yPos, 'e');
        }
      #endif
      break;
    // Facing East
    case EAST:
      mazeWalls[xPos][yPos + 1][1] = left;
      mazeWalls[xPos + 1][yPos][0] = front;
      mazeWalls[xPos][yPos][1] = right;
      #ifdef SIM
        if(left) {
          setWall(xPos, yPos, 'n');
        }
        if(front) {
          setWall(xPos, yPos, 'e');
        }
        if(right) {
          setWall(xPos, yPos, 's');
        }
      #endif
      break;
    // Facing South
    case SOUTH:
      mazeWalls[xPos + 1][yPos][0] = left;
      mazeWalls[xPos][yPos][1] = front;
      mazeWalls[xPos][yPos][0] = right;
      #ifdef SIM
        if(left) {
          setWall(xPos, yPos, 'e');
        }
        if(front) {
          setWall(xPos, yPos, 's');
        }
        if(right) {
          setWall(xPos, yPos, 'w');
        }
      #endif
      break;
    // Facing West
    case WEST:
      mazeWalls[xPos][yPos][1] = left;
      mazeWalls[xPos][yPos][0] = front;
      mazeWalls[xPos][yPos + 1][1] = right;
      #ifdef SIM
        if(left) {
          setWall(xPos, yPos, 's');
        }
        if(front) {
          setWall(xPos, yPos, 'w');
        }
        if(right) {
          setWall(xPos, yPos, 'n');
        }
      #endif
      break;
  }
}

// Rotates to a particular facing and moves one space or half space
void move(uint8_t dir, bool halfSpace) {
  if((facing == NORTH && dir == EAST) ||
     (facing == EAST && dir == SOUTH) ||
     (facing == SOUTH && dir == WEST) ||
     (facing == WEST && dir == NORTH)) {
      if(!halfSpace) {
        movingTurnRight();
      }
      else {
        turnRight();
        moveForwardHalf();
      }
     }
  else if((facing == NORTH && dir == SOUTH) ||
     (facing == EAST && dir == WEST) ||
     (facing == SOUTH && dir == NORTH) ||
     (facing == WEST && dir == EAST)) {
      turnRight();
      turnRight();
      if(!halfSpace) {
        moveForward();
      }
      else {
        moveForwardHalf();
      }
     }
  else if((facing == NORTH && dir == WEST) ||
     (facing == EAST && dir == NORTH) ||
     (facing == SOUTH && dir == EAST) ||
     (facing == WEST && dir == SOUTH)) {
      if(!halfSpace) {
        movingTurnLeft();
      }
      else {
        turnLeft();
        moveForwardHalf();
      }
     }
  else {
    if(!halfSpace) {
      moveForward();
    }
    else {
      moveForwardHalf();
    }
  }
  facing = dir;
}

// Determines direction to move, rotates to that direction, and moves forward (recalculates projected distances if no reasonable move found)
// Returns true if the goal has been reached, or false if not
bool navigate() {
  uint8_t dist = mazeNodes[xPos][yPos].dist;
  bool halfSpaces = false;
  #ifndef SIM
    if(!solving) {
      halfSpaces = true;
    }
  #endif
  solving = true;
  // If moving North lowers distance to goal, move North
  if(!mazeWalls[xPos][yPos + 1][1] && yPos + 1 < MAZE_HEIGHT && mazeNodes[xPos][yPos + 1].dist < dist) {
    move(NORTH, halfSpaces);
    yPos = yPos + 1;
  }
  // If moving East lowers distance to goal, move East
  else if(!mazeWalls[xPos + 1][yPos][0] && xPos + 1 < MAZE_WIDTH && mazeNodes[xPos + 1][yPos].dist < dist) {
    move(EAST, halfSpaces);
    xPos = xPos + 1;
  }
  // If moving West lowers distance to goal, move West
  else if(!mazeWalls[xPos][yPos][0] && mazeNodes[xPos][yPos].x >= 1 && mazeNodes[xPos - 1][yPos].dist < dist) {
    move(WEST, halfSpaces);
    xPos = xPos - 1;
  }
  // If moving South lowers distance to goal, move South
  else if(!mazeWalls[xPos][yPos][1] && mazeNodes[xPos][yPos].y >= 1 && mazeNodes[xPos][yPos - 1].dist < dist) {
    move(SOUTH, halfSpaces);
    yPos = yPos - 1;
  }
  // If end of maze has been reached (or start of maze has been reached while backtracking), switch backtracking mode
  if(mazeNodes[xPos][yPos].dist == 0) {
    backtracking = !backtracking;
    #ifndef SIM
      moveForwardHalf();
    #endif
    solving = false;
  }
  // If not backtracking, flood fill from center
  if(!backtracking) {
    int goal[4][2] = {{MAZE_WIDTH / 2, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2, MAZE_HEIGHT / 2 - 1}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2 - 1}};
    recalcMaze(goal, 4);
    return !solving;
  }
  // If backtracking, flood fill from start
  int goal[1][2] = {{0, 0}};
  recalcMaze(goal, 1);
  return !solving;
}

void createPath() {
  uint8_t dist;
  uint8_t x;
  uint8_t y;
  uint8_t i = 0;
  path[0] = &mazeNodes[xPos][yPos];
  // Rewrites the path variable using the maze info
  while(true) {
    // If moving North lowers distance to goal, add North node
    dist = path[i]->dist;
    x = path[i]->x;
    y = path[i]->y;

  // If moving North lowers distance to goal, add North node
  if(y + 1 < MAZE_HEIGHT && !mazeWalls[x][y + 1][1] && mazeNodes[x][y + 1].dist < dist) {
    path[i + 1] = &mazeNodes[x][y + 1];
  }
  // If moving East lowers distance to goal, add East node
  else if(x + 1 < MAZE_WIDTH && !mazeWalls[x + 1][y][0] && mazeNodes[x + 1][y].dist < dist) {
    path[i + 1] = &mazeNodes[x + 1][y];
  }
  // If moving West lowers distance to goal, add West node
  else if(x > 0 && !mazeWalls[x][y][0] && mazeNodes[x - 1][y].dist < dist) {
    path[i + 1] = &mazeNodes[x - 1][y];
  }
  // If moving South lowers distance to goal, add South node
  else if(y > 0 && !mazeWalls[x][y][1] && mazeNodes[x][y - 1].dist < dist) {
    path[i + 1] = &mazeNodes[x][y - 1];
  }
  // Breaks when there is no node with a lower score to add
  else
    break;
    i = i + 1;
  }
  #ifdef SIM
    clearAllColor();
    setColor(path[0]->x, path[0]->y, 'A');
    for(int j = 1; j < i; j++){
      setColor(path[j]->x, path[j]->y, 'C');
    }
    setColor(path[i]->x, path[i]->y, 'G');
  #endif

  pathEnd = i;
}
/*
// Moves the robot along the created path
void moveOnPath() {
  // Current node being considered on the path
  Node* current = path[0];
  // The direction of the current node being considered w/ resp. to robot
  enum cardinal_t direction;
  // Number of straight squares to move before turning
  int numSquares = 0;
  int i = 1;
  while(i <= pathEnd){
    if (current->x - 1 == path[i]->x) {
      direction = WEST;
    }else if (current->x + 1 == path[i]->x) {
      direction = EAST;
    }else if (current->y - 1 == path[i]->y) {
      direction = SOUTH;
    }else if (current->y + 1 == path[i]->y) {
      direction = NORTH;
    }else if (current->x == path[i]->x && current->y == path[i]->y) {
      logln("WARN: moveRobot called with current location\n");
      return;
    }

    // If next square is straight ahead, increment numSquares
    // And consider next square by incrementing i
    if(facing == direction){
      numSquares++;
      i++;
    }
    //If next square to be considered is not straight ahead, move numSquares
    else{
      // Moved forward numSquares, now rotate
      moveForward(numSquares);
      // rotateMove(direction);
      numSquares = 0;
      current = path[i - 1];

    }
  }
  // Makes the last move if the last node was straight ahead
  moveForward(numSquares + 1);
}
*/
// Does one iteration of the floodfill algorithm
void doRun() {
    updateWalls();
    /*
    #ifdef SIM
      createPath();
    #endif
    */
    navigate();
}