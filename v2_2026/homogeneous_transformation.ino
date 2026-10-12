// Homogeneous transformation - build one 4x4 DH transform from theta, d, r, alpha.
// theta and alpha are in degrees; d and r are lengths (mm).

BLA::Matrix<4, 4> dh_transform(float theta, float d, float r, float alpha) {

  float thetaRad = theta * pi / 180.0;
  float alphaRad = alpha * pi / 180.0;

  BLA::Matrix<4, 4> ax = {
    cos(thetaRad), -cos(alphaRad) * sin(thetaRad),  sin(alphaRad) * sin(thetaRad), r * cos(thetaRad),
    sin(thetaRad),  cos(thetaRad) * cos(alphaRad), -sin(alphaRad) * cos(thetaRad), r * sin(thetaRad),
    0,              sin(alphaRad),                  cos(alphaRad),                 d,
    0,              0,                              0,                             1
  };

  return ax;
}
