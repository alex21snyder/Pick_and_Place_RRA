// Open Gripper Function - open the end effector (servo 7) to release the object.
// Waits for the arm to settle, opens the gripper, then waits for the gripper to finish opening.
// pos7 is passed by reference so the caller's gripper position stays up to date.

void open_gripper(float& pos7) {
  delay(GRIPPER_PAUSE_MS);  // let the arm settle before releasing
  pos7 = GRIPPER_OPEN;
  servo7.write(pos7);       // open the gripper
  delay(GRIPPER_PAUSE_MS);  // place delay
}
