//Resolved Rates Algorithm(rra) - calculate sequential servo motor value to acheive specific end effector position and orientation.
//Take in information by passing it by refrence, and then change it in the function.

void rra(BLA::Matrix<6, 1>& myq, BLA::Matrix<6, 1>& myqr, BLA::Matrix<4, 4>& f1, BLA::Matrix<4, 4>& f2, BLA::Matrix<4, 4>& f3, BLA::Matrix<4, 4>& f4, BLA::Matrix<4, 4>& f5, BLA::Matrix<4, 4>& f6, BLA::Matrix<6, 6>& myJ, float(&myPosE), float(&myThetaE), BLA::Matrix<3, 1>& myXd, BLA::Matrix<3, 3>& myRd, const float(&mydelta_time), const float(&myvmin), const float(&myvmax), const float(&mywmin), const float(&mywmax), const float(&myepsilon_p), const float(&myepsilon_o), const float(&mylamda)) {

  while (1 == 1) {

    //Call kin function using the variables that are passed in by rra
    kin(myq, f1, f2, f3, f4, f5, f6, myJ);  //artifitial motor values, frames 1-6, jacobean

    // Find current position and orientation of end effector
    BLA::Matrix<3, 1> Xc = { f6(0, 3), f6(1, 3), f6(2, 3) };                                                              //current position
    BLA::Matrix<3, 3> Rc = { f6(0, 0), f6(0, 1), f6(0, 2), f6(1, 0), f6(1, 1), f6(1, 2), f6(2, 0), f6(2, 1), f6(2, 2) };  //current orientation

    // Calculate Position error (myPosE)
    BLA::Matrix<1, 1> step1m = ~(myXd - Xc) * (myXd - Xc);
    float step2 = step1m(0, 0);
    myPosE = sqrt(step2);

    // Calculate orientational error (myThataE)
    BLA::Matrix<3, 3> RotE = myRd * ~Rc;           //calculate rotational error
    step2 = RotE(0, 0) + RotE(1, 1) + RotE(2, 2);  //trace operation
    myThetaE = acos((step2 - 1) / 2);              //orientational error


    // *** If position and orientation desired thresholds are hit, break the loop and get out of rra function
    if (myPosE < myepsilon_p && myThetaE < myepsilon_o) {
      Serial.print("\nThis means the goal was hit!!!");
      break;
    }

    // Calculate Nhat
    float Xd_Xc[3][1];  //find differene between Xd-Xc
    Xd_Xc[0][0] = myXd(0, 0) - Xc(0, 0);
    Xd_Xc[1][0] = myXd(1, 0) - Xc(1, 0);
    Xd_Xc[2][0] = myXd(2, 0) - Xc(2, 0);
    float step1 = calculateNorm(Xd_Xc);            //norm(Xd-Xc)
    BLA::Matrix<3, 1> nhat = (myXd - Xc) / step1;  //Nhat

    // Calculate Vhat
    float vhat;
    step1 = ((myvmax - myvmin) * (myPosE - myepsilon_p)) / (myepsilon_p * (mylamda - 1));
    if (myPosE / myepsilon_p > mylamda) {
      vhat = myvmax;
    } else if (myPosE / myepsilon_p <= mylamda) {
      vhat = myvmin + step1;
    }

    // Calculate Pddot (derivative of position desired)
    BLA::Matrix<3, 1> Pddot = nhat * vhat;

    //CALCULATE W
    float w;
    step1 = ((mywmax - mywmin) * (myThetaE - myepsilon_o)) / (myepsilon_o * (mylamda - 1));
    if (myThetaE / myepsilon_o / myepsilon_p > mylamda) {
      w = mywmax;
    } else if (myThetaE / myepsilon_o / myepsilon_p <= mylamda) {
      w = mywmin + step1;
    }

    // Calculate Mhat
    step1 = 1 / (2 * sin(myThetaE));
    BLA::Matrix<3, 1> step3 = { RotE(2, 1) - RotE(1, 2), RotE(0, 2) - RotE(2, 0), RotE(1, 0) - RotE(0, 1) };
    BLA::Matrix<3, 1> mhat = step3 * step1;

    // Calculate Wd
    BLA::Matrix<3, 1> wd = mhat * w;

    // Calculate Xddot
    BLA::Matrix<6, 1> Xddot;
    Xddot(0, 0) = Pddot(0, 0);
    Xddot(1, 0) = Pddot(1, 0);
    Xddot(2, 0) = Pddot(2, 0);
    Xddot(3, 0) = wd(0, 0);
    Xddot(4, 0) = wd(1, 0);
    Xddot(5, 0) = wd(2, 0);

    // Jacobian pseudo inverse calculation
    BLA::Matrix<6, 6> step4 = myJ * ~myJ;  // mJ * transpose(myJ)
    float row = 1.0;
    for (int i = 0; i < 6; i++) {
      step4(i, i) = step4(i, i) + row;
    }
    step4 = Inverse(step4);                  //  (J*transpose(J)+row*I)^-1
    BLA::Matrix<6, 6> Jpinv = ~myJ * step4;  // pseudo inverse of jacobian

    // Qddot calculation
    BLA::Matrix<6, 1> qddot = Jpinv * Xddot;

    //Update artificial motor values (myq)
    float ratio = 180.0 / 3.14159;            // for converting rad to degrees
    BLA::Matrix<6, 1> inc = qddot * (ratio);  //increment matrix
    myq = myq + inc * mydelta_time;

    // Update real motor values (myqr)
    for (int i = 0; i < 6; i++) {
      myqr(i, 0) = myqr(i, 0) + inc(i, 0) * mydelta_time * 9.0;  //we mult by 9 here bc, 1 degree = 9 units
    }

    // Command joints (servo motors) to go updated position
    servo1.writeMicroseconds(myqr(0, 0));
    servo2.writeMicroseconds(myqr(1, 0));
    servo3.writeMicroseconds(myqr(2, 0));
    servo4.writeMicroseconds(myqr(3, 0));
    servo5.writeMicroseconds(myqr(4, 0));
    servo6.writeMicroseconds(myqr(5, 0));

  }  // end of big while loop

}  //END OF FUNCTION