// Close Gripper Function - close the end effector (servo 7) to pick up the object.
// Waits for the arm to settle, closes the gripper, then waits for the gripper to finish closing.
// pos7 is passed by reference so the caller's gripper position stays up to date.

void close_gripper(float& pos7) {
  delay(GRIPPER_PAUSE_MS);  // let the arm settle before gripping
  pos7 = GRIPPER_CLOSED;
  servo7.write(pos7);       // close the gripper
  delay(GRIPPER_PAUSE_MS);  // pick delay
}
