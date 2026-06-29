#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Math.h>
Adafruit_PWMServoDriver board_1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver board_2 = Adafruit_PWMServoDriver(0x40);

String mode = "";

const int NUMBER_SERVOS_LEFT =9;
const int NUMBER_SERVOS_RIGTH =9;

const int servoChannel[NUMBER_SERVOS_LEFT] = {0,1,2,3,4,5,6,7,8};  // Pins των servo στο PCA9685 1)tibia = 0 2)femur = 1 3)coxa = 2 για ολα τα ποδια
const double initial_pos[NUMBER_SERVOS_LEFT]  = {180,0,90,0,180,90,0,180,90};//τα πρωτα 3 ειναι για το ενα αριστερο ποδι ενω τα αλλα δυο 3αρια ειναι για τα δυο δεξια ποδια-->πανε στο board 1
                                              //     L1      L2       L3
const double initial_pos_rigth[NUMBER_SERVOS_RIGTH] = {0,180,90,180,0,90,180,0,90};//τα πρωτα 3 ειναι για το ενα δεξι ποδι ενω τα αλλα δυο 3αρια ειναι για τα δυο αριστερα ποδια-->πανε στο board 2
                                                    //    L4        L5       L6
const int servoMinPulse = 100;  // περίπου 1.0 ms σε ticks PCA9685 (50Hz)
const int servoMaxPulse = 730;  

const double pi = 3.1415927;
const double a3 = 131.92;
const double a2 = 79.91;//  mm
const double a4 = 45.57;

struct Point {
 double x;//members στη σωστη σειρα
 double y;
 double z;
};

Point Point_start = {135,-50,70};//x,y,z οπως τα βαλαμε στο structure
Point Control_point_1 = {135,-50,-20};//xE[50,140],zE[-100,100],yE(-200.200)
Point Control_point_2 = {145,50,-20};
Point Point_end = {135,50,70};//Τα βρηκαμε με forward kinematics

Point Point_start_z = {135,0,100};
Point Control_point_1_z = {135,0,-20};
Point Control_point_2_z = {135,0,-20};
Point Point_end_z = {135,0,100};

Point Point_start_x = {200,0,70};
Point Control_point_1_x = {200,0,70};
Point Control_point_2_x = {50,0,70};
Point Point_end_x = {50,0,70};


void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo Control - MG996R");

  board_1.begin();
  board_1.setPWMFreq(50);  // Θέτουμε τη συχνότητα στα 50Hz (20ms κύκλος)
  board_2.begin();
  board_2.setPWMFreq(50);
  delay(50);

    for (int i = 0;i<NUMBER_SERVOS_LEFT ;i++){
  setServoAngle(servoChannel[i],initial_pos[i]);
  setServoAngle_rigth(servoChannel[i],initial_pos_rigth[i]);
  delay(15);
  }
  delay(1000);
  Serial.println("Type mode:");
}

void cubic_bezier_linear_bezier(Point P0,Point P1,Point P2,Point P3){   //BEZIER--->Rigth LINEAR--->LEFT
  
  for (int i = 0; i <= 100; i=i+10) {
    
    double t = i/100.0;//ΠΡΑΓΜΑΤΙΚΗ ΔΙΑΙΡΕΣΗ
    double xo = pow((1-t),3)*P0.x+3*pow((1 - t),2)* t * P1.x +3*(1-t)*pow(t,2)*P2.x+pow(t,3)*P3.x;
    double yo = pow((1-t),3)*P0.y+3*pow((1 - t),2)* t * P1.y +3*(1-t)*pow(t,2)*P2.y+pow(t,3)*P3.y;
    double zo = pow((1-t),3)*P0.z+3*pow((1 - t),2)* t * P1.z +3*(1-t)*pow(t,2)*P2.z+pow(t,3)*P3.z;
    
    double x1 = (1-t)*P3.x+ t * P0.x ;//end_point(P3)---->start_point(P0)
    double y1 = (1-t)*P3.y+ t * P0.y ;
    double z1 = (1-t)*P3.z+ t * P0.z ;

    IK_rigth(xo, yo, zo);//rigth
    IK_linear(x1,y1,z1);//left
    
    //delay(10);
}
}

void cubic_bezier_linear_bezier_x(Point P0,Point P1,Point P2,Point P3){   //BEZIER--->Rigth LINEAR--->LEFT
  
  for (int i = 0; i <= 100; i=i+10) {
    
    double t = i/100.0;//ΠΡΑΓΜΑΤΙΚΗ ΔΙΑΙΡΕΣΗ
    double xo = pow((1-t),3)*P0.x+3*pow((1 - t),2)* t * P1.x +3*(1-t)*pow(t,2)*P2.x+pow(t,3)*P3.x;
    double yo = pow((1-t),3)*P0.y+3*pow((1 - t),2)* t * P1.y +3*(1-t)*pow(t,2)*P2.y+pow(t,3)*P3.y;
    double zo = pow((1-t),3)*P0.z+3*pow((1 - t),2)* t * P1.z +3*(1-t)*pow(t,2)*P2.z+pow(t,3)*P3.z;
    
    double x1 = (1-t)*P3.x+ t * P0.x ;//end_point(P3)---->start_point(P0)
    double y1 = (1-t)*P3.y+ t * P0.y ;
    double z1 = (1-t)*P3.z+ t * P0.z ;

    double x2 = (1-t)*P0.x+ t * P3.x ;//start_point(P0)---->end_point(P3)
    double y2 = (1-t)*P0.y+ t * P3.y ;
    double z2 = (1-t)*P0.z+ t * P3.z ;

    IK_rigth(xo, yo, zo);//rigth
    IK_L1(x2,y2,z2);
    IK_linear(x1,y1,z1);//left
    
    delay(10);
}
}

void cubic_bezier(Point P0,Point P1,Point P2,Point P3){
  for (int i = 0; i <= 100; i=i+10) {
    
    double t = i/100.0;//ΠΡΑΓΜΑΤΙΚΗ ΔΙΑΙΡΕΣΗ
    double xo = pow((1-t),3)*P0.x+3*pow((1 - t),2)* t * P1.x +3*(1-t)*pow(t,2)*P2.x+pow(t,3)*P3.x;
    double yo = pow((1-t),3)*P0.y+3*pow((1 - t),2)* t * P1.y +3*(1-t)*pow(t,2)*P2.y+pow(t,3)*P3.y;
    double zo = pow((1-t),3)*P0.z+3*pow((1 - t),2)* t * P1.z +3*(1-t)*pow(t,2)*P2.z+pow(t,3)*P3.z;

    IK_rigth(xo, yo, zo);//rigth
    IK_linear(xo,yo,zo);//left
}
}

void linear_bezier_cubic_bezier(Point P0,Point P1,Point P2,Point P3){  //BEZIER--->Left LINEAR--->Rigth
  
  for (int i = 0; i <= 100; i=i+10) {
    
    double t = i/100.0;//ΠΡΑΓΜΑΤΙΚΗ ΔΙΑΙΡΕΣΗ
    double xo = pow((1-t),3)*P0.x+3*pow((1 - t),2)* t * P1.x +3*(1-t)*pow(t,2)*P2.x+pow(t,3)*P3.x;
    double yo = pow((1-t),3)*P0.y+3*pow((1 - t),2)* t * P1.y +3*(1-t)*pow(t,2)*P2.y+pow(t,3)*P3.y;
    double zo = pow((1-t),3)*P0.z+3*pow((1 - t),2)* t * P1.z +3*(1-t)*pow(t,2)*P2.z+pow(t,3)*P3.z;
    
    double x1 = (1-t)*P3.x+ t * P0.x ;
    double y1 = (1-t)*P3.y+ t * P0.y ;
    double z1 = (1-t)*P3.z+ t * P0.z ;

    IK_rigth(x1, y1, z1);//rigth
    IK_linear(xo,yo,zo);//left
    
    //delay(10);
}
}

//safe_acos μας περιοριζει τις γωνιες
double safe_acos(double x) {
  if (x < -1.0) x = -1.0;
  if (x > 1.0) x = 1.0;
  return acos(x);
}
void setServoAngle(int channel, int angle) {
  //περιορισμος γωνιας
  angle = constrain(angle, 0, 270);
  int pulseLength = map(angle, 0, 270, servoMinPulse, servoMaxPulse);
  board_1.setPWM(channel, 0, pulseLength);  //board 1
  }

void setServoAngle_rigth(int channel, int angle){
  angle = constrain(angle, 0, 270);
  int pulseLength = map(angle, 0, 270, servoMinPulse, servoMaxPulse);
  board_2.setPWM(channel, 0, pulseLength);  //board 2
}

void IK_linear(double xo,double yo,double zo){
 double theta1_1 = atan2(yo,xo);
 double theta1_2 =90*pi/180-theta1_1;

 double r3 = cos(19.21)*a4;
 double rx = sqrt(pow(xo,2)+pow(yo,2))-r3;
 double r = sqrt(pow(rx,2)+pow(zo,2));

 double f1 = safe_acos((pow(a2,2)+pow(r,2)-pow(a3,2))/(2*a2*r));
 double f2 = atan2(zo,rx);
 double theta2_1 = f2-f1; 
 double theta2_2 = 90*pi/180+theta2_1;

 double zeta = safe_acos((pow(a2,2)+pow(a3,2)-pow(r,2))/(2*a2*a3));
 double theta3_1 = pi-zeta;
 double theta3_2 = 180*pi/180-theta3_1;
 
 double  angle_degrees[NUMBER_SERVOS_LEFT] = {theta3_2*180/pi,theta2_2*180/pi,theta1_2*180/pi, 
                                              (theta3_1-25*pi/180)*180/pi,(90*pi/180-theta2_1)*180/pi,(90*pi/180+theta1_1)*180/pi, 
                                              (theta3_1-25*pi/180)*180/pi,(90*pi/180-theta2_1)*180/pi,(90*pi/180+theta1_1)*180/pi};//left
 for (int i = 0;i<NUMBER_SERVOS_LEFT ;i++){
  setServoAngle(servoChannel[i],angle_degrees[i]);
  delay(7);
  }
}
void IK_rigth(double xo,double yo,double zo){
 double theta1_1 = atan2(yo,xo);
 double theta1_2 =90*pi/180-theta1_1;

 double r3 = cos(19.21)*a4;
 double rx = sqrt(pow(xo,2)+pow(yo,2))-r3;
 double r = sqrt(pow(rx,2)+pow(zo,2));

 double f1 = safe_acos((pow(a2,2)+pow(r,2)-pow(a3,2))/(2*a2*r));
 double f2 = atan2(zo,rx);
 double theta2_1 = f2-f1; 
 double theta2_2 = 90*pi/180+theta2_1;

 double zeta = safe_acos((pow(a2,2)+pow(a3,2)-pow(r,2))/(2*a2*a3));
 double theta3_1 = pi-zeta;
 double theta3_2 = 180*pi/180-theta3_1;
 
 double  angle_degrees_rigth[NUMBER_SERVOS_RIGTH] = {(theta3_1-25*pi/180)*180/pi,(90*pi/180-theta2_1)*180/pi,(90*pi/180+theta1_1)*180/pi,
                                                     theta3_2*180/pi,theta2_2*180/pi,theta1_2*180/pi,
                                                     theta3_2*180/pi,theta2_2*180/pi,theta1_2*180/pi};

 for (int i = 0;i<NUMBER_SERVOS_LEFT ;i++){
  setServoAngle_rigth(servoChannel[i],angle_degrees_rigth[i]);
  delay(7);
  }
}
void IK_L1(double xo,double yo,double zo){
 double theta1_1 = atan2(yo,xo);
 double theta1_2 =90*pi/180-theta1_1;

 double r3 = cos(19.21)*a4;
 double rx = sqrt(pow(xo,2)+pow(yo,2))-r3;
 double r = sqrt(pow(rx,2)+pow(zo,2));

 double f1 = safe_acos((pow(a2,2)+pow(r,2)-pow(a3,2))/(2*a2*r));
 double f2 = atan2(zo,rx);
 double theta2_1 = f2-f1; 
 double theta2_2 = 90*pi/180+theta2_1;

 double zeta = safe_acos((pow(a2,2)+pow(a3,2)-pow(r,2))/(2*a2*a3));
 double theta3_1 = pi-zeta;
 double theta3_2 = 180*pi/180-theta3_1;
 
 double  angle_degrees[NUMBER_SERVOS_LEFT] = {theta3_2*180/pi,theta2_2*180/pi,theta1_2*180/pi, };//left

 for (int i = 0;i<servoChannel[3] ;i++){
  setServoAngle(servoChannel[i],angle_degrees[i]);
  delay(7);
  }
}


void loop() {
  if (Serial.available() > 0){
   mode = Serial.readStringUntil('\n');
   mode.trim();

 if (mode == "F"){
   for(int k=0;k<=180;k++){
  cubic_bezier_linear_bezier(Point_start,Control_point_1,Control_point_2,Point_end);
  linear_bezier_cubic_bezier(Point_start,Control_point_1,Control_point_2,Point_end);
  delay(10);
   }
 }
 else if (mode == "B"){
for(int k=0;k<=180;k++){
  cubic_bezier_linear_bezier(Point_end,Control_point_2,Control_point_1,Point_start);
  linear_bezier_cubic_bezier(Point_end,Control_point_2,Control_point_1,Point_start);
  delay(10);
   }   
 }
 else if (mode == "P"){
   for(int k=0;k<=180;k++){
    delay(10);
   }
 }
 else if (mode == "D"){
   for(int k=0;k<=180;k++){
     cubic_bezier(Point_end_z,Control_point_2_z,Control_point_1_z,Point_start_z);
     delay(10000);
   }
 }
 else if (mode == "U"){
   for(int k=0;k<=180;k++){
     cubic_bezier(Point_start_z,Control_point_1_z,Control_point_2_z,Point_end_z);
     delay(10000);
   }
 }
 else if (mode == "R"){
   for(int k=0;k<=180;k++){
   cubic_bezier_linear_bezier_x(Point_start_x,Control_point_1_x,Control_point_2_x,Point_end_x);
   delay(10);
   }
 }
 }
}



