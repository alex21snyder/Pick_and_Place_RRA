// might have to make the q their vector as well but unsure at this time.!!!!!!!!!!


// kin takes in : frames1-6, q values, and J value. It spits out new frame values and J values
void kin(BLA::Matrix<6, 1>& myq, BLA::Matrix<4, 4>& f1, BLA::Matrix<4, 4>& f2, BLA::Matrix<4, 4>& f3, BLA::Matrix<4, 4>& f4, BLA::Matrix<4, 4>& f5, BLA::Matrix<4, 4>& f6, BLA::Matrix<6, 6>& myj) {

//dh table (in radians and mm)
  BLA::Matrix<6, 4> DH = { myq(0,0), 35.5, -4, 90,         
                           myq(1,0) + 270, -8.7, -269, 180,   
                           myq(2,0) + 270, -8.7, 0, 90,
                           myq(3,0), 110.2, -5.5, 90,
                           myq(4,0), 25.5, 0, 270,
                           myq(5,0), 55, 11.5, 0 };

  //  homogenous transformations
  BLA::Matrix<4, 4> a1 = { cos(DH(0, 0) * pi / 180.0), -cos(DH(0, 3) * pi / 180.0) * sin(DH(0, 0) * pi / 180.0), sin(DH(0, 3) * pi / 180.0) * sin(DH(0, 0) * pi / 180.0), DH(0, 2) * cos(DH(0, 0) * pi / 180.0),
                           sin(DH(0, 0) * pi / 180.0), cos(DH(0, 0) * pi / 180.0) * cos(DH(0, 3) * pi / 180.0), -sin(DH(0, 3) * pi / 180.0) * cos(DH(0, 0) * pi / 180.0), DH(0, 2) * sin(DH(0, 0) * pi / 180.0),
                           0, sin(DH(0, 3) * pi / 180.0), cos(DH(0, 3) * pi / 180.0), DH(0, 1),
                           0, 0, 0, 1 };

  BLA::Matrix<4, 4> a2 = { cos(DH(1, 0) * pi / 180.0), -cos(DH(1, 3) * pi / 180.0) * sin(DH(1, 0) * pi / 180.0), sin(DH(1, 3) * pi / 180.0) * sin(DH(1, 0) * pi / 180.0), DH(1, 2) * cos(DH(1, 0) * pi / 180.0),
                           sin(DH(1, 0) * pi / 180.0), cos(DH(1, 0) * pi / 180.0) * cos(DH(1, 3) * pi / 180.0), -sin(DH(1, 3) * pi / 180.0) * cos(DH(1, 0) * pi / 180.0), DH(1, 2) * sin(DH(1, 0) * pi / 180.0),
                           0, sin(DH(1, 3) * pi / 180.0), cos(DH(1, 3) * pi / 180.0), DH(1, 1),
                           0, 0, 0, 1 };

  BLA::Matrix<4, 4> a3 = { cos(DH(2, 0) * pi / 180.0), -cos(DH(2, 3) * pi / 180.0) * sin(DH(2, 0) * pi / 180.0), sin(DH(2, 3) * pi / 180.0) * sin(DH(2, 0) * pi / 180.0), DH(2, 2) * cos(DH(2, 0) * pi / 180.0),
                           sin(DH(2, 0) * pi / 180.0), cos(DH(2, 0) * pi / 180.0) * cos(DH(2, 3) * pi / 180.0), -sin(DH(2, 3) * pi / 180.0) * cos(DH(2, 0) * pi / 180.0), DH(2, 2) * sin(DH(2, 0) * pi / 180.0),
                           0, sin(DH(2, 3) * pi / 180.0), cos(DH(2, 3) * pi / 180.0), DH(2, 1),
                           0, 0, 0, 1 };

  BLA::Matrix<4, 4> a4 = { cos(DH(3, 0) * pi / 180.0), -cos(DH(3, 3) * pi / 180.0) * sin(DH(3, 0) * pi / 180.0), sin(DH(3, 3) * pi / 180.0) * sin(DH(3, 0) * pi / 180.0), DH(3, 2) * cos(DH(3, 0) * pi / 180.0),
                           sin(DH(3, 0) * pi / 180.0), cos(DH(3, 0) * pi / 180.0) * cos(DH(3, 3) * pi / 180.0), -sin(DH(3, 3) * pi / 180.0) * cos(DH(3, 0) * pi / 180.0), DH(3, 2) * sin(DH(3, 0) * pi / 180.0),
                           0, sin(DH(3, 3) * pi / 180.0), cos(DH(3, 3) * pi / 180.0), DH(3, 1),
                           0, 0, 0, 1 };
  BLA::Matrix<4, 4> a5 = { cos(DH(4, 0) * pi / 180.0), -cos(DH(4, 3) * pi / 180.0) * sin(DH(4, 0) * pi / 180.0), sin(DH(4, 3) * pi / 180.0) * sin(DH(4, 0) * pi / 180.0), DH(4, 2) * cos(DH(4, 0) * pi / 180.0),
                           sin(DH(4, 0) * pi / 180.0), cos(DH(4, 0) * pi / 180.0) * cos(DH(4, 3) * pi / 180.0), -sin(DH(4, 3) * pi / 180.0) * cos(DH(4, 0) * pi / 180.0), DH(4, 2) * sin(DH(4, 0) * pi / 180.0),
                           0, sin(DH(4, 3) * pi / 180.0), cos(DH(4, 3) * pi / 180.0), DH(4, 1),
                           0, 0, 0, 1 };

  BLA::Matrix<4, 4> a6 = { cos(DH(5, 0) * pi / 180.0), -cos(DH(5, 3) * pi / 180.0) * sin(DH(5, 0) * pi / 180.0), sin(DH(5, 3) * pi / 180.0) * sin(DH(5, 0) * pi / 180.0), DH(5, 2) * cos(DH(5, 0) * pi / 180.0),
                           sin(DH(5, 0) * pi / 180.0), cos(DH(5, 0) * pi / 180.0) * cos(DH(5, 3) * pi / 180.0), -sin(DH(5, 3) * pi / 180.0) * cos(DH(5, 0) * pi / 180.0), DH(5, 2) * sin(DH(5, 0) * pi / 180.0),
                           0, sin(DH(5, 3) * pi / 180.0), cos(DH(5, 3) * pi / 180.0), DH(5, 1),
                           0, 0, 0, 1 };

  //  frames calculations 

  //frames_1=a1
  BLA::Matrix<4, 4> frames_2 = a1 * a2;
  BLA::Matrix<4, 4> frames_3 = frames_2 * a3;
  BLA::Matrix<4, 4> frames_4 = frames_3 * a4;
  BLA::Matrix<4, 4> frames_5 = frames_4 * a5;
  BLA::Matrix<4, 4> frames_6 = frames_5 * a6;//pos1 = pos1 + apos1


// Update the passed in frame value to the value calculated in this funciton.
  for (int i = 0; i < 4; i++) {  
    for (int j = 0; j < 4; j++) {
      f1(i,j) = a1(i,j);
    }
  }
  for (int i = 0; i < 4; i++) {  
    for (int j = 0; j < 4; j++) {
      f2(i,j) = frames_2(i,j);
    }
  }
  for (int i = 0; i < 4; i++) {  
    for (int j = 0; j < 4; j++) {
      f3(i,j) = frames_3(i,j);
    }
  }
  for (int i = 0; i < 4; i++) {  
    for (int j = 0; j < 4; j++) {
      f4(i,j) = frames_4(i,j);
    }
  }
  for (int i = 0; i < 4; i++) {  
    for (int j = 0; j < 4; j++) {
      f5(i,j) = frames_5(i,j);
    }
  }
  for (int i = 0; i < 4; i++) {  
    for (int j = 0; j < 4; j++) {
      f6(i,j) = frames_6(i,j);
    }
  }
 

  //                            JACOBIAN START

  BLA::Matrix<3, 1> d1 = { a1(0, 3), a1(1, 3), a1(2, 3) };
  BLA::Matrix<3, 3> r1 = { a1(0, 0), a1(0, 1), a1(0, 2), a1(1, 0), a1(1, 1), a1(1, 2), a1(2, 0), a1(2, 1), a1(2, 2) };

  BLA::Matrix<3, 1> d2 = { frames_2(0, 3), frames_2(1, 3), frames_2(2, 3) };
  BLA::Matrix<3, 3> r2 = { frames_2(0, 0), frames_2(0, 1), frames_2(0, 2), frames_2(1, 0), frames_2(1, 1), frames_2(1, 2), frames_2(2, 0), frames_2(2, 1), frames_2(2, 2) };

  BLA::Matrix<3, 1> d3 = { frames_3(0, 3), frames_3(1, 3), frames_3(2, 3) };
  BLA::Matrix<3, 3> r3 = { frames_3(0, 0), frames_3(0, 1), frames_3(0, 2), frames_3(1, 0), frames_3(1, 1), frames_3(1, 2), frames_3(2, 0), frames_3(2, 1), frames_3(2, 2) };

  BLA::Matrix<3, 1> d4 = { frames_4(0, 3), frames_4(1, 3), frames_4(2, 3) };
  BLA::Matrix<3, 3> r4 = { frames_4(0, 0), frames_4(0, 1), frames_4(0, 2), frames_4(1, 0), frames_4(1, 1), frames_4(1, 2), frames_4(2, 0), frames_4(2, 1), frames_4(2, 2) };

  BLA::Matrix<3, 1> d5 = { frames_5(0, 3), frames_5(1, 3), frames_5(2, 3) };
  BLA::Matrix<3, 3> r5 = { frames_5(0, 0), frames_5(0, 1), frames_5(0, 2), frames_5(1, 0), frames_5(1, 1), frames_5(1, 2), frames_5(2, 0), frames_5(2, 1), frames_5(2, 2) };

  BLA::Matrix<3, 1> d6 = { frames_6(0, 3), frames_6(1, 3), frames_6(2, 3) };
  BLA::Matrix<3, 3> r6 = { frames_6(0, 0), frames_6(0, 1), frames_6(0, 2), frames_6(1, 0), frames_6(1, 1), frames_6(1, 2), frames_6(2, 0), frames_6(2, 1), frames_6(2, 2) };

  BLA::Matrix<3, 1> vec = { 0.0, 0.0, 1.0 };
  BLA::Matrix<3, 3> r0 = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };



  // get a b c d e f values for jacobian (see matlab mark up) (top half of J values)
  BLA::Matrix<3, 1> ansa;
  cross(r0 * vec, d6, ansa);

  BLA::Matrix<3, 1> ansb;
  cross(r1 * vec, d6 - d1, ansb);

  BLA::Matrix<3, 1> ansc;
  cross(r2 * vec, d6 - d2, ansc);

  BLA::Matrix<3, 1> ansd;
  cross(r3 * vec, d6 - d3, ansd);

  BLA::Matrix<3, 1> anse;
  cross(r4 * vec, d6 - d4, anse);

  BLA::Matrix<3, 1> ansf;
  cross(r5 * vec, d6 - d5, ansf);

  //populate J with all new values (top half)

  // for ansa
  for (int i = 0; i < 3; i++) {
    myj(i, 0) = ansa(i);
  }
  // for ansb
  for (int i = 0; i < 3; i++) {
    myj(i, 1) = ansb(i);
  }
  for (int i = 0; i < 3; i++) {
    myj(i, 2) = ansc(i);
  }
  for (int i = 0; i < 3; i++) {
    myj(i, 3) = ansd(i);
  }
  for (int i = 0; i < 3; i++) {
    myj(i, 4) = anse(i);
  }
  for (int i = 0; i < 3; i++) {
    myj(i, 5) = ansf(i);
  }

  // bottom half J fill in
  BLA::Matrix<3, 1> ansg;
  ansg = r0 * vec;
  for (int i = 3; i < 6; i++) {
    myj(i, 0) = ansg(i-3);
  }

  BLA::Matrix<3, 1> ansh;
  ansh = r1 * vec;
  for (int i = 3; i < 6; i++) {
    myj(i, 1) = ansh(i-3);
  }

BLA::Matrix<3, 1> ansi;
  ansi = r2 * vec;
  for (int i = 3; i < 6; i++) {
    myj(i, 2) = ansi(i-3);
  }
BLA::Matrix<3, 1> ansj;
  ansj = r3 * vec;
  for (int i = 3; i < 6; i++) {
    myj(i, 3) = ansj(i-3);
  }
BLA::Matrix<3, 1> ansk;
  ansk = r4 * vec;
  for (int i = 3; i < 6; i++) {
    myj(i, 4) = ansk(i-3);
  }

BLA::Matrix<3, 1> ansl;
  ansl = r5 * vec;
  for (int i = 3; i < 6; i++) {
    myj(i, 5) = ansl(i-3);
  }










}  //end of program