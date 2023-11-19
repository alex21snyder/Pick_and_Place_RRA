//Cross Product Function - calculate the cross product between two 3X1 shaped matricies.
//Take in information by passing it by refrence, and then change it in the function.


void cross(const BLA::Matrix<3, 1>& vectorA, const BLA::Matrix<3, 1>& vectorB, BLA::Matrix<3, 1>& result) {
  result(0, 0) = vectorA(1, 0) * vectorB(2, 0) - vectorA(2, 0) * vectorB(1, 0);
  result(1, 0) = vectorA(2, 0) * vectorB(0, 0) - vectorA(0, 0) * vectorB(2, 0);
  result(2, 0) = vectorA(0, 0) * vectorB(1, 0) - vectorA(1, 0) * vectorB(0, 0);
}
