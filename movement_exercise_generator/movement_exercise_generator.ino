  /* MOVEMENT EXERCISE GENERATOR
   * Requirements:
   * - Two axes: x and y
   * - Same movement cannot follow itself
   * - Same movement can only be done again when countermovement has been done
   * --> only two possible movements to  be done next every time
   * 
   * Directions: e.g. 0=up, 1=down, 2=right, 3=left
  */

  // Just some starting values
  int next_y = 0
  int prev_y = 1
  int next_x = 2
  int prev_x = 3
  
  int next_movement = -1
  int axis = -1
  
void setup() {
  // put your setup code here, to run once:
}

void loop() {

  axis = rand() % 2 // pick random axis

  // y-axis
  if (axis == 0) {
    next_movement = next_y
    next_y.swap(prev_y)     // swap the directions so next coming is the opposite
  }
  // x-axis
  else {
    next_movement = next_x
    next_x.swap(prev_x)     // swap the directions so next coming is the opposite    
  }

  // Call to movement instruction function with (next_movement) here. 

}
