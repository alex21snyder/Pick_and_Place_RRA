//rra function !!! lets get thsi money babby


void rra(BLA::Matrix<6, 1>& myq, BLA::Matrix<6, 1>& myqr, BLA::Matrix<4, 4>& f1, BLA::Matrix<4, 4>& f2, BLA::Matrix<4, 4>& f3, BLA::Matrix<4, 4>& f4, BLA::Matrix<4, 4>& f5, BLA::Matrix<4, 4>& f6, BLA::Matrix<6, 6>& myJ, float(&myPosE), float(&myThetaE), BLA::Matrix<3, 1>& myXd, BLA::Matrix<3, 3>& myRd, const float(&mydelta_time), const float(&myvmin), const float(&myvmax), const float(&mywmin), const float(&mywmax), const float(&myepsilon_p), const float(&myepsilon_o), const float(&mylamda)) {

  int i = 1;
  // Start Loop     (need to add i++ as last line if using while loop)
  while (1 == 1) {
    //for (int i = 1; i <= 200; i++) {  // in the end will be the while loop but for resting use for loop

    //Call kin function using the variables that are passed in by rra
    kin(myq, f1, f2, f3, f4, f5, f6, myJ);

    // Xc calculation
    BLA::Matrix<3, 1> Xc = { f6(0, 3), f6(1, 3), f6(2, 3) };  //Define Xc

    // Define Rc
    BLA::Matrix<3, 3> Rc = { f6(0, 0), f6(0, 1), f6(0, 2), f6(1, 0), f6(1, 1), f6(1, 2), f6(2, 0), f6(2, 1), f6(2, 2) };

    // Calculate Position error (PosE) (PosE=sqrt((Xd-Xc).'*(Xd-Xc));%poition error)
    BLA::Matrix<1, 1> step1m = ~(myXd - Xc) * (myXd - Xc);  // (Xd-Xc).'*(Xd-Xc)
    float step2 = step1m(0, 0);                             // change to float
    myPosE = sqrt(step2);                                   //take square root to get positional error

    //Calculate orientation error (RotE)    Thetae=acos((trace(Re)-1)/2); %orientation error
    //step1 = RE = Rd* trans(Rc) calc rotational error
    //step2= trace of RE
    //step3 = calc Orientation error. !! using the built in acos is 3x as slow as making your own function.
    BLA::Matrix<3, 3> RotE = myRd * ~Rc;           //step 1
    step2 = RotE(0, 0) + RotE(1, 1) + RotE(2, 2);  // step 2
    myThetaE = acos((step2 - 1) / 2);              //orientational error



    // If goal is hit end program
    // if (myPosE < myepsilon_p && myThetaE < myepsilon_o) {
    if (myPosE < myepsilon_p && myThetaE < myepsilon_o) {
      Serial.print("\n");
      Serial.print("This means the goal was hit!!!");
      break;
    }

    //CALCULATE NHAT

    //find Xd-Xc and make it a 3x1 vector not using library
    float Xd_Xc[3][1];
    Xd_Xc[0][0] = myXd(0, 0) - Xc(0, 0);
    Xd_Xc[1][0] = myXd(1, 0) - Xc(1, 0);
    Xd_Xc[2][0] = myXd(2, 0) - Xc(2, 0); /*
//print Xd-Xc
  Serial.println("\n"); // Newline
  for (int i = 0; i < 3; i++) {
    Serial.print(Xd_Xc[i][0]);
    Serial.print("\n"); // Tab-separated
  }
  Serial.println(); 
*/
    float step1 = calculateNorm(Xd_Xc);  // this is norm(Xd-Xc)
    //Serial.print(step1);
    BLA::Matrix<3, 1> nhat = (myXd - Xc) / step1;  //nhat answer
    //Serial.print("\n");
    //Serial << nhat;


    // CALCULATE VHAT
    float vhat;
    step1 = ((myvmax - myvmin) * (myPosE - myepsilon_p)) / (myepsilon_p * (mylamda - 1));
    if (myPosE / myepsilon_p > mylamda) {
      vhat = myvmax;
    } else if (myPosE / myepsilon_p <= mylamda) {
      vhat = myvmin + step1;
    }

    //CALCULATE Pddot
    BLA::Matrix<3, 1> Pddot = nhat * vhat;

    //CALCULATE W
    float w;
    step1 = ((mywmax - mywmin) * (myThetaE - myepsilon_o)) / (myepsilon_o * (mylamda - 1));
    if (myThetaE / myepsilon_o / myepsilon_p > mylamda) {
      w = mywmax;
    } else if (myThetaE / myepsilon_o / myepsilon_p <= mylamda) {
      w = mywmin + step1;
    }

    //CALCULATE MHAT
    step1 = 1 / (2 * sin(myThetaE));
    BLA::Matrix<3, 1> step3 = { RotE(2, 1) - RotE(1, 2), RotE(0, 2) - RotE(2, 0), RotE(1, 0) - RotE(0, 1) };

    //Serial.print("\n");
    //Serial << step3;
    BLA::Matrix<3, 1> mhat = step3 * step1;

    //CALCULATE WD
    BLA::Matrix<3, 1> wd = mhat * w;
    //Serial.print("\n");
    //Serial << wd;




    // CALCULATE Xddot
    BLA::Matrix<6, 1> Xddot;

    Xddot(0, 0) = Pddot(0, 0);
    Xddot(1, 0) = Pddot(1, 0);
    Xddot(2, 0) = Pddot(2, 0);
    Xddot(3, 0) = wd(0, 0);
    Xddot(4, 0) = wd(1, 0);
    Xddot(5, 0) = wd(2, 0);

    //Serial.print("\n");
    //Serial << Xddot;

    // J PINV CALC!!!

    BLA::Matrix<6, 6> step4 = myJ * ~myJ;
    float row = 1.0;  //step 4 is now J*J.'+row*I ///////////see if i can bring this down to .1 then .01 if poss
    for (int i = 0; i < 6; i++) {
      step4(i, i) = step4(i, i) + row;
    }
    step4 = Inverse(step4);  //  (J*J.'+row*I)^-1
    //Serial.print("\n");
    //Serial << step4;
    //Serial.print("\n");
    //Serial.print(step4(3,3),4);

    // i think i can call the whole thing jpinv if i want to save memory.
    BLA::Matrix<6, 6> Jpinv = ~myJ * step4;  //this is Jpinv!!
    /*
    Serial.print("\n");
    Serial << Jpinv;
    Serial.print("\n");
    Serial.print(Jpinv(0,0),4);
*/
    // QDDOT Qddot calculation
    BLA::Matrix<6, 1> qddot = Jpinv * Xddot;
    //Serial.print("\n");
    //Serial << qddot;
    //Serial.print("\n");
    //Serial.print(qddot(3, 0), 4);


    // UPDATE Q CALCULATION
    float blah = 180.0 / 3.14159; // for converting rad to degrees
    BLA::Matrix<6, 1> inc = qddot * (blah);
    myq = myq + inc * mydelta_time;

    //UPDATE REAL MOTOR (could make this loop with if statement !!!!!!!!!!!)
    //myqr = myqr + inc * mydelta_time; // comment this out.
    myqr(0,0) =myqr(0,0) + inc(0,0) * mydelta_time*9.0;
    myqr(1,0) =myqr(1,0) + inc(1,0) * mydelta_time*11.0; //we mult by 5 here because 1 real degree = 5 B degrees///
    myqr(2,0) =myqr(2,0) + inc(2,0) * mydelta_time*9.0; //we mult by 5 here because 1 real degree = 5 B degrees///
    //myqr(2,0) =myqr(2,0) + inc(2,0) * mydelta_time;
    myqr(3,0) =myqr(3,0) + inc(3,0) * mydelta_time*9.0;
    myqr(4,0) =myqr(4,0) + inc(4,0) * mydelta_time*9.0;
    myqr(5,0) =myqr(5,0) + inc(5,0) * mydelta_time*9.0;




    // could make a if statemtnt here like if its one of the first fe witerations , dalay .250 micro seconds or soemthing like that


    // COMMAND MOTOR TO GO TO SPECIFIC POINT (this might be for onl integers but we will see) (could loop this as well!!!!!)
    servo1.writeMicroseconds(myqr(0,0));  // starting position when power is turned on
    //servo2.write(myqr(1,0));
    servo2.writeMicroseconds(myqr(1,0)); //980 is straight up and down (R=44, A=0, B=980)
    servo3.writeMicroseconds(myqr(2,0)); //980 is straight up and down (R=44, A=0, B=980)
    //servo3.write(myqr(2,0));
    servo4.writeMicroseconds(myqr(3,0));
    servo5.writeMicroseconds(myqr(4,0));
    servo6.writeMicroseconds(myqr(5,0));
    //servo7.write(pos7);


    //OUTPUT ALL RELEVANT VALUES
/*
    // display i here.
    Serial.print("\n");
    Serial.print("Iteration #");
    Serial.print(i);
    //display myq here
    Serial.print("\n");
    Serial.print("qfake =");
    Serial << myq;
    
    //disp position error (PosE) here
    Serial.print("\n");
    Serial.print("position error = ");  // this is pose e before we run this run of rra.
    Serial.print(myPosE);
    //display orientation error here (ThetaE)
    Serial.print("\n");
    Serial.print("orientational error = ");  // this is pose e before we run this run of rra.
    Serial.print(myThetaE);
    
    // disp real q value for motors here (myqr)
    Serial.print("\n");
    Serial.print("qactual =");
    Serial << myqr;


*/









    i++;
    // if (i==20){
    // break;
    //}
    //break;  //get me out of the while 1==1 function

  }  // end of big while loop
}  //END OF FUNCTION