/*
Function to print the current joint angles to the arduino output screen
*/



void print_state(const RobotState& state) {

  Serial.print("\n");
  Serial.print(state.q);  // display q matrix
  Serial.print("\n");

}
