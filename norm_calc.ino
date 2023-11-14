float calculateNorm(float vector[3][1]) {
  float sumOfSquares = 0.0;

  // Calculate the sum of squares of each element.
  for (int i = 0; i < 3; i++) {
    sumOfSquares += vector[i][0]* vector[i][0];
  }

  // Calculate the square root of the sum of squares to find the norm.
  float norm = sqrt(sumOfSquares);

  return norm;
}
