/* Robotic Arm Using RRA Main File  $ALEX SNYDER$

by: Alex Snyder
8/xx/23

*/

#include <Servo.h>
#define pi 3.14159265
#include <BasicLinearAlgebra.h>

using namespace BLA;
Servo servo1;  // create servo object to control a servo
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

// all variables before void setup are global variables and can be accessed by anywhere in the program
void setup() {
  delay(3000);  //this makes the serial print only print once for some reason (should be 1000)
  Serial.begin(9600);

  // REAL POSITION OF EACH Q real position of each motor
  //made this float but servo library only takes int lol
  float pos1 = 1401.0;  // the range is from 0 to 180
  //float pos2 = 44.0;  //the range is -90 to 180
  float pos2 = 995.0; //980 is straight up and down (R=44, A=0, B=980) //maybe this value is 990 instead. technically should be 995.
  float pos3 = 1511.1; //91 is og
  float pos4 = 1500.0; //90 og
  float pos5 = 1500.0;  //90 og
  float pos6 = 1500.0;  //90 og
  float pos7 = 30.0;  //open , 62 is closed tight.

  // ARTIFICIAL POSITION OF EACH Q artificial position of each q
  float apos1 = 0.0;
  float apos2 = 0.0;
  float apos3 = 0.0;
  float apos4 = 0.0;
  float apos5 = 0.0;
  float apos6 = 0.0;
/*
  // BARTIFICIAL POSITION OF EACH Q artificial position of each q (B)
  float bpos1 = 1.0*apos1;
  float apos2 = 0.0;
  float apos3 = 0.0;
  float apos4 = 0.0;
  float apos5 = 0.0;
  float apos6 = 0.0;
*/

  servo1.writeMicroseconds(pos1);  // starting position when power is turned on
  //servo2.write(pos2);
  servo2.writeMicroseconds(pos2); //980 is straight up and down (R=44, A=0, B=980)
  //servo3.write(pos3);
  servo3.writeMicroseconds(pos3);
  servo4.writeMicroseconds(pos4);
  servo5.writeMicroseconds(pos5);
  servo6.writeMicroseconds(pos6);
  servo7.write(pos7);

  servo1.attach(11);
  servo2.attach(2);
  servo3.attach(3);
  servo4.attach(4);
  servo5.attach(5);
  servo6.attach(6);
  servo7.attach(7);
  delay(1000);  //!!!! DONT KNOW IF THIS DELAY IS NECC

  //artificial q values
  BLA::Matrix<6, 1> q = { apos1, apos2, apos3, apos4, apos5, apos6 };

  //real q values
  BLA::Matrix<6, 1> qr = { pos1, pos2, pos3, pos4, pos5, pos6 };  // real values to send to motors

  //  initialize the frames.
  BLA::Matrix<4, 4> frames_1 = { 0, 0, 0, 0,
                                 0, 0, 0, 0,  // could make this cleaner, did it for seeing where
                                 0, 0, 0, 0,  // everthing is.
                                 0, 0, 0, 0 };
  BLA::Matrix<4, 4> frames_2 = { 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0 };

  BLA::Matrix<4, 4> frames_3 = { 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0 };
  BLA::Matrix<4, 4> frames_4 = { 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0 };
  BLA::Matrix<4, 4> frames_5 = { 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0 };
  BLA::Matrix<4, 4> frames_6 = { 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0,
                                 0, 0, 0, 0 };
  // initiliaze jacobian, Position error and Orientation error
  BLA::Matrix<6, 6> J;
  J.Fill(0);
  float PosE; //Position error
  float ThetaE; //Orientation error



  //!!!!!!!!!!!!!!!!!!!!!!    PROGRAM BEGIN !!!!!!!!!!!!!!!!!!!!!

  //STARTING PARAMETERS
  float delta_time = .028;  //s (same as matlab was .1) //could increase if i want //.05 //////.1 is what i think im going w for video .18 // .022
  float vmin = 7;          //m/s (.31 scaling ratio) /9.3
  float vmax = 24;         //m/s (.31 scaling ratio) /31
  float wmin = .30;        //rad/s (same as matlab) .0349
  float wmax = .55;        //rad/s (same as matlab) maybe increase this // .1745
  float epsilon_p = 1;     // m, could make this number bigger if i want  was .31;
  float epsilon_o = .1;    //rad was .0524;      /.1 is considered og
  float lamda = 5;         // unitless so I will use the samea as matlab


  //DISPLAY STARTING VALUES
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("Starting Q(artificial) values =");
  Serial.print("\n");
  Serial << q;
  Serial.print("\n");

  //call kinematics function
  //kin(q, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J);  // to save memory, i dont need to run this yet, will run in rra only

  //LOCATION 1 (go from home to location 1)
  //Xd
  BLA::Matrix<3, 1> Xd = { 188.0, -104.0, 312.0 };
  //Rd
  BLA::Matrix<3, 3> Rd = { 0, 0, 1.0, 0, 1.0, 0, -1.0, 0, 0 };

  //RRA from home to 1
  rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);
  delay(500);
  pos7 = 63;  //closed
  servo7.write(pos7);
  //delay(1000);  //CHANGE TO 1000/5000

  //LOCATION 2 (go to location 2 from location 1)
  //Xd
  Xd = { 104, 0, 394.0 };
  //Rd
  Rd = { 0.7071, 0, 0.7071, 0, 1.0, 0, -0.7071, 0, 0.7071 };
  rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);
  //delay(1000);




  //LOCATION 3 (go to location 3 from location 2)
  //Xd
  Xd = { 190.0, 104.0, 305.0 };
  //Rd
  Rd = { 0, 0, 1.0, 0, 1.0, 0, -1.0, 0, 0 };
  wmin = .75;    //rad/s (same as matlab) .0349
  wmax = 1.375;  //rad/s (same as matlab) maybe increase this // .1745
  //RRA from 2 to 3
  rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);
  delay(500);
  pos7 = 30;  //open
  servo7.write(pos7);
  //delay(1000);  //CHANGE TO 1000/5000

delay(500);  //!!!! DONT KNOW IF THIS DELAY IS NECC


   //  3 TO HOME
  BLA::Matrix<6, 1> qh = { pos1, pos2, pos3, pos4, pos5, pos6 };
  BLA::Matrix<6, 1> qdiff = qh - qr;
  Serial.print("\n");
  Serial << qdiff;
  float amount = 2000.0;
  qdiff = qdiff / amount;
  for (int j = 0; j < amount; j++) {
    qr = qr + qdiff;
    servo1.writeMicroseconds(qr(0, 0));  // starting position when power is turned on
    servo2.writeMicroseconds(qr(1, 0));
    servo3.writeMicroseconds(qr(2, 0));
    servo4.writeMicroseconds(qr(3, 0));
    servo5.writeMicroseconds(qr(4, 0));
    servo6.writeMicroseconds(qr(5, 0));
    //Serial.print("\n");
    //Serial << qr;
    delay(5);
  }

Serial.print("\n");

Serial.print("done");




  //HOME LOCATION
  //Xd
  Xd = { 2.0, -25.50, 469.70 };
  //Rd
  Rd = { 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0 };
  epsilon_p = 1;  // m, could make this number bigger if i want  was .31;
  epsilon_o = .1;
  vmin = 9.3;  //m/s (.31 scaling ratio) /9.3
  vmax = 31;
  wmin = .30;  //rad/s (same as matlab) .0349
  wmax = .55;  //rad/s (same as matlab) maybe increase this // .1745
  //rra from 3 to home
  //rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);
  //delay(3000);  // check final q values

  //go back to exact home values
  pos1 = 79.0;  // the range is from 0 to 180
  pos2 = 44.0;  //the range is -90 to 180
  pos3 = 91.0;
  pos4 = 90.0;
  pos5 = 90.0;
  pos6 = 90.0;
  pos7 = 30.0;  //30

  servo1.write(pos1);  // starting position when power is turned on
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);
  servo5.write(pos5);
  servo6.write(pos6);
  servo7.write(pos7);

  //Serial.print("\n");
  //Serial << J;
  //Serial.print("\n");
  //Serial << frames_6;

  //Serial << Xd;

  /////////////////////////////////


  //TESTING

  /*
  pos7 = 55.0;  //30
  servo7.write(pos7); // closed
  delay(1000);
  pos7 = 30.0;  //30
  servo7.write(pos7); //open
  delay(5000);
  pos7 = 63.0;    //closed
  servo7.write(pos7); 
  delay(5000);
  pos7 = 30.0;     //open
  servo7.write(pos7); 
  
// home info

 //Xd
  Xd = { 2.0, -25.50, 469.70 };
  //Rd
  Rd = { 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0 };





  Serial.print("\n");
Serial << frames_1;
Serial.print("\n");
Serial << frames_2;
Serial.print("\n");
Serial << frames_3;
Serial.print("\n");
Serial << frames_4;
Serial.print("\n");
Serial << frames_5;
Serial.print("\n");
Serial << frames_6;
*/


}  // end of program !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*
                USEFUL STUFF FOR TESTING/ DEBUGGING PROGRAM
// PRINTING ALL THE FRAMES
Serial.print("\n");
Serial << frames_1;
Serial.print("\n");
Serial << frames_2;
Serial.print("\n");
Serial << frames_3;
Serial.print("\n");
Serial << frames_4;
Serial.print("\n");
Serial << frames_5;
Serial.print("\n");
Serial << frames_6;

float alex[4][4] = {
    { 9.9, 1, 1, 1 },
    { 2, 2, 2, 2 },
    { 3, 3, 3, 3 },
    { 4, 4, 4, 4 }
  };
  float bob[4][4] = {
    { 1, 1, 1, 1 },
    { 2, 2, 2, 2 },
    { 3, 3, 3, 3 },
    { 4, 4, 4, 200.2 },
  };

float ans[4][4] = {
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
  };




*/



void loop() {

  while (1 == 1) {  // END OF PROGRAM (do nothing loop)
  }
}





/*
REV ##

rev 01- has motor 1-7 working.
rev 03- try to get kin functino up
rev07-Working on The Kin Function 10/26 
rev 08, i think im just gonna modify 6 different arrays.
rev 10 - working kinematics
rev 11- cleaned up version of 10 (not code change)
rev 13 - I will be changing the code to use the stdl instead.
rev 16- starting over using linear algebra bit.
rev 17- kin/jacoian =$$ (it looks like no memroy is leaking here)
rev 18- redeffined important variables to global variables (this means that most of these pass by refrence functiosn are uselses)
(see notes on 10/31 for explination)
rev 19- 1st attempted at rra function (lord help me)
rev 20- fuck it im starting over with no global variablles. Its bad practice acording to the internet. I will erase all the 
motor stuff as well and I can refrence rev 01 or rev 02 or rev 03 when I want to actually encoperate the motors.
rev 22- fuck it gotta redo the whole thing in m not mm
rev 23- fuck it we goin back to mm for the whole thing.
rev 27 - first goal was hit! (it converged and it looks like the q values make sense.)
rev 28 - lets try and see if we can choose the og starting posoition and get it to converge.
rev 29 - these initial parameters are for 0 to 1. they are optimized pretty well.
it takes 11 seconds right now and can be adjusted with delta_t. This Xd and Rd are for demo only and not my acualy positions.
can make delta 1 = .2 seconds for 6 second total time if i want
rev 30 - rra trial works
rev 31 - demo 00
rev 32 -write the full alg, test it, video it, see what i like and dont like and go from there.
rev 33 - do new alg at end bit. First working Simulation!!!!!!
rev 34- try to reduce computijng time.
rev 35 - hahahahahaah. the calculations take 2 fucking seconds each. lmao. It was the fucking serial monitor.
thats fucking crazy. I am really happy I decided to meditate and really try and analize why this is taking so long to run
and taking things out and running the code. The fucking Serial monitor lol.
Ok next step- decrease the delta_t in the formula i have, and add delays after each itteration in the rra. This will be
the real delta t. maybe if I decrease the delta t a fuck ton it will be really smooth.
I will redo the whole thing with different parameters. Delete all the serial monitor for each rra itteration when testing.


NOTES:

home values:
motors tweaking in begining- might have to change batteries.
delay for motor should be longer than 15, maybe 30 or 45 

make a file that uses if statments to see if the motor angles are not 90 degrees, if they arent inch one degree toward 90 or straight up angle. maybe make the motors change values dratsically at qs(espeicallyq1)
so that the motors can plow through with no issue

if the variable is int, i can only go to whole number which may be issue

VIDEO: james bond music during the loud dun nut da da show flashes of the whole robot then black. during begining
of song show from bottom up starting at like 25 seconds ish

high all the time for it doing stuff oooorrrr daft punk doing it right robtic beat

*/


/*
USEFUL INFORMATION:

1. USEFUL FUNCTIONS

// hello world function
void hi(){
  Serial.print("Hello, World!");
  Serial.print("\n");
}

//infinate loop to end program
void inf(){
  while (1==1) {
}
}

2. FUNCTION NOTES
- If something is repeating, its good to make it a function, if not just keep
it in the main. 
-Function should only be 20 lines of code max. if you
need to make functions inside functions
-functions should do one thing and do them well
-If statments and loops should ideally be in different function files and
called

https://workat.tech/machine-coding/tutorial/design-good-functions-classes-clean-code-86h68awn9c7q#:~:text=A%20function%20should%20do%20one,doing%20more%20than%20one%20thing.

3. CLEAN CODE
-"Clean code always looks like it was writtten by someone who cares."
-Michael Feathers


4. VARIABLES
int=-2,147,483,648 to 2,147,483,647
*/

/*
%{ 
!!!!!!!!!!!!!!!!!!!!!!!!  PROGRESS / JOURNAL  !!!!!!!!!!!!!!!!!!!!!!!!

9/8
-was able to retrace my steps and learn what I did to get the rotation
matrix Desired.
-Spent a ton of time on trying to correct my dh tables by artificially
flipping the motor 3. When I did that I was still getting the same frame 6.
And like before when I changed the 269 to -269, it seems like everything
position wise works out.

Ive pretty thouroughly checked my answer that I get with this script against
the know values I get with the Rev04 which is known to be correct for what
I am doing. I am 92% sure that even though I changed one value in the dh
table which makes no sense, the end effector location, and orientation in
the last frame seem to be correct. with e 92% confidence in my final
frame value(this means that all the frames beloew it should also be
correct) I have no choice but to push foward and assume that these dh
frames are correct. 

My next step should be implementing this kinematic calcultaor on arduino,
then maybe picking desired positions and orientations I Want achieved, then
start legit programming this thing.

My next step is getting this to be a arduino funtion then working on the
rest of the sarudino script. maybe have the robot arm go to a position to
the right at 90 degrees.
_____________________________________________________________________________

9/11/23
Got the DH A1 table part working. Next steps, finish DH table with all a values,
then if you want make it a function or hold off on that. Then comes making positions 
desired and orientations desired, and copy the rest of the program to see if it works without motors,
then make it work with motors.

9/13/23
Poss start learning c++ to learn how to decrease memory consumed

next work on multiplying the arrays out prob new variable for each frame unles i can figure out 3 dim array

9/14
Wow, c++ is something else lol. Spent an absurd amount of time just making a set of scripts that multiply
4x4 arrays. Should be able to implement this into the main now with ease though so I can finish the
transofrmation matricies and the jacobeans to finish the kinematics part of the robot. Gonna be like 
20 functions but fuck it lol.

These files are saved under mult function. They have been tested against matlab.

REDO or use vectors<vectors> v {blah blah} for arrays from now on

10/17
-for vmax vmin and all other parameters.
-use a ratio of link lengths to calculate them.
-ex avg of previous link lengths/previous parameters(do each one seperately) = avg of current link lengths over x parameter
-solve for x(single paremeter i am looking for), then do this for the rest of the parameters.


10/26
-I think I could make functions inside of functions to create cleaner code
-I think I will start with just a fuck ton of functions, then maybe clean it up if i need.
-in order to change array using function, you must pass it by refrence and return nothing.
-Left off, I just got a1 to come through. Maybe print the entire a1 to test we good
-then move onto all a values
-then try to finish all of kin stuff

10/27
-Kinametics are finally done!
-wow that was alot harder than i thought it would be lol. That arduino language (c++), without being able to use
the c++ standard library is annoying. I made some really nice functions though and I was able to use the 
pass by refrence thing I learned about in my classes, and I think it worked great and saved alot of memory sapce on the arduino.

Next I will do jacobean, then RRA(lord help me).
I think I will make second function for jacobean which is diff than matlab. I think it will be clearer.
First I will clean up what I have though

10/28
-Just figured out you can use an a version of the stl thing that will allow you to return macricies lmao.
-I think my plan is to redo the whole thing with this stl bit so that it will be alot eaiser to work on 
these complex equations.

-yup, see sketch in arm_testing called sketch_oct_28 sketch and copy this format pls :)


10/30
-After alot of messing around I decided that using the standard library wasnt gonna work.
-After I declared like 4 of my matricies, the dynamic memory shot up to like 50% and,
the serial monitor started going balisitic. 
-I think it has to do with the memory.
-Myu new plan is to try it without using the standard libarry/
- need to figure out how to make these complex functions work.

-implemented the kinematics using the new library and it came out really good.
Really happy I decided to switch to this one.

-Next time attack jacobian and rra.
-to test rra maybe I can append either the position or the q values after each iteration to check them!
-This will be great becasue when i switch to implementing it on the arm, I can just add the delay here after each iteration.
-I could also upend the each error to a matrix to track ther error throughtout as well. This will be great.
-Also change the q values to match the liear algebra format like the rest of the matricies.
-Also might have to make the kin function change f1 to frame_1 in the kin section.

10/31

-concers: I am pretty confident that rewriting over variables does not leak memory or cuase the amount
of memory used to grow over time. If not I will be in trouble lmao.

-got the jacobian up and working fuck yea. now onto the dreaded RRA alg :0
-each iteration is a new q value

-I dont think I have decided on a rotd and a q desired so I will just make up a simple one
for testing.
-So evertyhing is w repect to the base frame right?

-I just learned that everything declared before the void function is declared as a global variable. They can be accessed by
any function in the entire program. This also leads to a drastic increase in dynamic memory used.
If a variable is declared in a functino, it can only be used in that specific function.
Also! if a variable is a global variable, it does not need to be passed by through a functionm its already availbale to be accessed.
If dynamic memory becomes an issue, I should just make certain things not global variables.

11/1/23
-frames look to be a touch off. I will chalk it up to eyeballing the measurments.
-for Rd i will use the same as current and for Xd, I will choose 61.2,-6, 419.7.

11/2/23
-I just fixed the matlab simulation so it works like a charm now. 
It works with the values we were given in the table an it is so smooth now lol.
-I will copy thing exactly for my project

11/3
- psuedo inverse is just inverse but it allows inverse on non square matricies and special cases 
like non invertable matricies.


11/4
-fuck yea bro i was fucking right.
-Plugged in the inland mega board and it worked how it should!
-Im pretty confident I was just at the threshold of some time of memory I was not able to see(some type of ram).
This arduino mega is a fucking units.
-LFG

-27, pass in rotation error and orientation error
-29, correcting dh table.
*/

// -ALEX 23
