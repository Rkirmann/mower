//////////Pins/////////////////////
//measure
#define voltPin A0
#define ledPin A1
#define currentPin A2
#define currentPin2 A3
//
#define revleft 2
#define fwdleft 3
#define revright 4
#define fwdright 5
#define cutter 10
//IR
//#define RECV_PIN 12
//UH
#define UH1 6 //trig
#define UH2 7 //echo//

#include <EEPROM.h>
////////////////////////////BUTTON//////////////////////////////////
#include <RBD_Timer.h>
#include <RBD_Button.h>

RBD::Button addFencebutton(12); // input_pullup by default
bool firstcoord = false;
RBD::Button startbutton(11); // input_pullup by default
//////////////////////////////////////////////////////////////////





//////////TIME/////////////////////
unsigned long time = millis();
unsigned long amptime = millis();
unsigned long mowertime = millis();
unsigned long gotime = millis();
unsigned long blinktime = millis();
//////////TIME/////////////////////

////////////////////////  geofence  ////////////////////////////////////////

int npol; // massiivi suurus
long xp[25];//*xp; //idakraadid fence list
long yp[25];//*yp; //pohjakraadid fence list

struct Geofence{
int npol;
long yp[25];//*yp; //pohjakraadid fence list
long xp[25];//*xp; //idakraadid fence list
};
Geofence fence;
int inFence = 0;
int speedcount = 0;
float avgspeed = 0;
bool avoidfence = false;
bool turninfence = false;
////////////////////////geofence////////////////////////////////////////


////////////////////////  MOTORS /////////////////////////////////////
bool resetTimer = false;
bool resetTimer2 = false;
bool resetTimer3 = false;
int direction = 1;

///////////////////////////////////////////////////////////////////////////////



/////////////////////////////// UH  ///////////////////////////////////////
#include <Ultrasonic.h>
Ultrasonic ultrasonic(UH1, UH2);
int distance;
bool avoidUH = false;
/////////////////////////////////Current//////////////////////////////////
#define CURRENT_SAMPLES 10
int currentsum = 0;                    // sum of samples taken
int currentsample_count = 0; // current sample number
float current = 0.0;
float current2 = 0.0;
bool highcurrent = false;
bool highcurrent2 = false;
bool avoidcurrent = false;
///////////////////////////////////////////////////////////////////////////

///////////////////////////////Voltage/////////////////////////////////////
//using voltage divider
//resistor 1 = 1   MegaOhm
//resistor 2 = 100 KiloOhm
// number of analog samples to take per reading
#define VOLT_SAMPLES 10
int voltsum = 0;                    // sum of samples taken
unsigned char voltsample_count = 0; // current sample number
float voltage;
///////////////////////////////Voltage/////////////////////////////////////





///////////////////////////////GPS//////////////////////////////////////
#include <NMEAGPS.h>
#include <GPSport.h>
NMEAGPS gps; // This parses the GPS characters
gps_fix fix; // This holds on to the latest values
long y = 0;
long x = 0;
float km = 0;
 
//////////////////////////////////////////////////////////////////////














int wire1;
int wire2;
int wire3;
#include <Wire.h>
void receiveEvent(int howMany) {
  wire1 = Wire.read();    // receive byte as an integer
  wire2 = Wire.read();
  wire3 = Wire.read();
}

void setup() {

  //WIRE
 Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  
  
 //pinMode(LED_BUILTIN, OUTPUT);
  //////gps///////////
  DEBUG_PORT.begin(9600);
  while (!Serial)
    ;
  gpsPort.begin(9600);
  ////////////gps/////////////

  // Debug console
  Serial.begin(9600);


  
  EEPROM.get(0, fence);

  npol = fence.npol;
  for (int i = 0; i < npol; i++)
  {
    yp[i] = fence.yp[i];

    Serial.print("yp: ");
    Serial.print(yp[i]);
    Serial.print(" ");

    xp[i] = fence.xp[i];
    Serial.print("xp: ");
    Serial.print(xp[i]);
    Serial.print(" ");
    Serial.println();
  }
  Serial.print("npol: ");
  Serial.println(npol);
 
  
  
  //pinMode(voltPin, INPUT);
  pinMode(currentPin, INPUT);

  /////////////motor/////////////////////////////////////////
    
    pinMode(revleft, OUTPUT);      // set Motor pins as output
    pinMode(fwdleft, OUTPUT);
    pinMode(revright, OUTPUT);
    pinMode(fwdright, OUTPUT);


    pinMode(cutter, OUTPUT);
    
    
  /////////////motor/////////////////////////////////////
  pinMode(voltPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(currentPin, INPUT);
  pinMode(currentPin2, INPUT);


  

}










bool start = false;
bool cutstart = false;
void loop()
{
  
  
  if (wire1 == 1)
   digitalWrite(LED_BUILTIN, HIGH);  
  if (wire1 == 0)
  digitalWrite(LED_BUILTIN, LOW);  



  getDistance(); 
  getCoords();
  addFence();

  //measureVoltage();      //get voltage from voltage divider r1=1Mohm, r2=100Kohm, takes 10ns
  //Serial.print(voltage);
  //Serial.println (" V");
  //low voltage
  /* 
    if (voltage <= 11.8) {
     
    }
  */

  current = measureCurrent(currentPin);       //get current from 0.1ohm shunt. takes 10ns
  current2 = measureCurrent(currentPin2);       //get current from 0.1ohm shunt. takes 10ns

  //Serial.println(current);
  //Serial.println(current2);
  
  if (startbutton.onPressed()){
    gotime = millis();
    //digitalWrite(LED_BUILTIN, HIGH);
    start= true;
    cutstart= true;
  }

  
  //start auto drive
  if (millis() - gotime > 10000 && start==true && wire1 == 0){  
    moveRobot();         
  }

  //manual drive
  if (wire1 == 1 && wire2 == 100){
    direction = 1;
    turn();
    digitalWrite(ledPin, HIGH);
  }
  else if (wire1 == 1 && wire2 == 0){
    direction = -1;
    turn();
    digitalWrite(ledPin, HIGH);
  }
  else if (wire1 == 1 && wire3 == 100){
    direction = 1;
    drive();
    digitalWrite(ledPin, HIGH);
  }
  else if (wire1 == 1 && wire3 == 0){
    direction = -1;
    drive();
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    stop();
    digitalWrite(ledPin, LOW);
  }
  

  ////////////////////////////    CUTTER      /////////////////////////////////////////////////
  
  //start cutter
  
  if (cutstart == true){
    digitalWrite(cutter, HIGH);
    cutstart = false;
  }
   
    
    //check if cutter jammed
    if (current2 > 1.7 && highcurrent2 == false && millis() - gotime > 5000 && start==true && millis() - mowertime > 2000){               
         
        highcurrent2 = true;
        mowertime = millis();
       
    }
    
    if ( millis() - mowertime > 1000  && current2 > 1.7 && highcurrent2 == true ){
     //digitalWrite(ledPin, HIGH);
       digitalWrite(cutter, LOW);
    }
    if ( millis() - mowertime >= 8000 && highcurrent2 == true){
       //digitalWrite(ledPin, LOW);
      digitalWrite(cutter, HIGH);
      highcurrent2 = false;
      mowertime = millis();
    }
    


}













void addFence() {
  
  if (addFencebutton.onPressed()) {
    if (firstcoord == false){
    npol = 0;
    firstcoord = true; 
    }
    xp[npol] = x;
    yp[npol] = y;
    
    
    fence.xp[npol] = xp[npol];
    fence.yp[npol] = yp[npol];

    npol += 1;
    fence.npol = npol;

    //print
    Serial.print("added to fence:");
    Serial.print(fence.npol);
    Serial.print(" , ");
    Serial.print(fence.xp[npol]);
    Serial.print(" , ");
    Serial.println(fence.yp[npol]);


    EEPROM.put(0, fence);
  }//if
}


void getDistance(){
  int dist = ultrasonic.read();
  for (int i = 0; i < 4; i++){
    dist += ultrasonic.read(); 
  }
  distance = dist/5;
  
}//dist

void getCoords(){
  while (gps.available(gpsPort))
  {
    
    fix = gps.read();
    getFence();         //kasuta getCoords geofence arvutamiseks    
    //Serial.println(inFence);
    
    //DEBUG_PORT.print( F("Location: ") );
    if (fix.valid.location)
    {

      digitalWrite(ledPin, HIGH);
      blinktime = millis();
      y = (fix.latitudeL()); // integer displayco
      //Blynk.virtualWrite(V0, y);
      //DEBUG_PORT.print(y);
      //DEBUG_PORT.print( ',' );
      x = (fix.longitudeL()); // integer display
      //Blynk.virtualWrite(V1, x);
      //DEBUG_PORT.println(x);
    }//if
    
      
    
    
    /* 
    if (fix.valid.speed) {
      km += fix.speed_kph();
      speedcount += 1;
      if (speedcount == 3){
        avgspeed = km /3;
        Serial.print("avg speed: ");
        Serial.println(avgspeed);
        km = fix.speed_kph();
        speedcount = 1;
      }
      
      //DEBUG_PORT.print(fix.speed_kph());
      //  DEBUG_PORT.println(" kmh");
      //Blynk.virtualWrite(V3, fix.speed_kph());
    }//speed
    */


  } //while
  if (millis() - blinktime > 500)
      digitalWrite(ledPin, LOW);
} //getcoords


//////////////////////////  MOVE ROBOT  //////////////////////////////////////////
void drive (){
  if (direction == 1){
    digitalWrite(fwdright, HIGH); // move forward
    digitalWrite(revright, LOW);
    digitalWrite(fwdleft, HIGH);
    digitalWrite(revleft, LOW);
  }
  else {
    digitalWrite(fwdright, LOW); // move forward
    digitalWrite(revright, HIGH);
    digitalWrite(fwdleft, LOW);
    digitalWrite(revleft, HIGH);
  }
}

void stop (){
  digitalWrite(fwdright, LOW); //Stop
  digitalWrite(revright, LOW);
  digitalWrite(fwdleft, LOW);
  digitalWrite(revleft, LOW);
}

void turn (){
  if (direction == 1){
  digitalWrite(fwdright, HIGH); //turn left
  digitalWrite(revright, LOW);
  digitalWrite(revleft, HIGH);
  digitalWrite(fwdleft, LOW);
  } else {
  digitalWrite(fwdright, LOW); //turn right
  digitalWrite(revright, HIGH);
  digitalWrite(revleft, LOW);
  digitalWrite(fwdleft, HIGH);  
  }
}

void testcurrent (){
  if (current > 1.3 && highcurrent == false){
    amptime = millis(); 
    highcurrent = true;  
  }

  if (current > 1.3 && millis() - amptime > 200 && highcurrent == true){
      avoidcurrent = true;
      highcurrent = false;
      direction *= -1;
      resetTimer = false;
  }
  
  
}//test current

//test UH
void testUH(){
  if (distance <= 20){
    avoidUH = true;
    
  }
}

void testfence(){
  if (inFence == 0){
    
    avoidfence = true;
  }
}




void moveRobot(){

  
  testUH();
  testfence();
  testcurrent();
  //Serial.println(current);

//all ok movement
//inFence == 1 &&
 if (current2 > 1.3 && highcurrent2 == false){
   stop();
 }
 else if (avoidcurrent == false && avoidfence == false && avoidUH == false) {
    direction = 1;
    drive();
    resetTimer = false;
      resetTimer2 = false;
      resetTimer3 = false;
      avoidcurrent = false;
      avoidUH = false;
      avoidfence = false;
      turninfence = false;
  }

///current trigger
  else if (avoidcurrent == true){
     
    if (resetTimer == false){
      resetTimer = true;
      time = millis(); //and reset time
    }
    if (millis() - time < 500)
    {
      stop();
    }
    if (millis() - time >= 500 && millis() - time < 2500)
    { //reverse direction
      drive();
    }
    if (millis() - time >= 2500 && millis() - time < 3000)
    { 
      stop();
    }
    if (millis() - time >= 3000 && millis() - time < 7000)
    { //reverse direction
      turn();
    }
    if (millis() - time >= 7000 && millis() - time < 7500)
    { //reverse direction
      stop();
    }
    if (millis() - time >= 7500)
    { 
      resetTimer = false;
      resetTimer2 = false;
      resetTimer3 = false;
      avoidcurrent = false;
      avoidUH = false;
      avoidfence = false;
      turninfence = false;
    }
  } //if current trigger
///UH trigger
  else if (avoidUH==true /*&& avoidfence == false */){
     direction = -1;
    if (resetTimer2 == false){
    resetTimer2 = true;
    time = millis(); //and reset time
    }
    if (millis() - time < 500)
    {
      stop();
    }
    if (millis() - time >= 500 && millis() - time < 2500)
    { //reverse direction
      drive();
    }
    if (millis() - time >= 2500 && millis() - time < 3000)
    { 
      stop();
    }
    if (millis() - time >= 3000 && millis() - time < 7000)
    { //reverse direction
      turn();
    }
    if (millis() - time >= 7000 && millis() - time < 7500)
    { //reverse direction
      stop();
    }
    if (millis() - time >= 7500)
    { 
      resetTimer = false;
      resetTimer2 = false;
      resetTimer3 = false;
      avoidcurrent = false;
      avoidUH = false;
      avoidfence = false;
      turninfence = false;
    }
  } //if UH trigger
// fence trigger
  else if (avoidfence == true){
    direction = -1;
    if (resetTimer3 == false){
      resetTimer3 = true;
      time = millis(); //reset time
    }
    if (millis() - time < 500 && turninfence == false){
      stop();
    }
    if (millis() - time >= 500 && turninfence == false){ 
      drive();
    }
    if (millis() - time >= 20000 && millis() - time < 27000 && turninfence == false){ 
      turn();
    }

    if (inFence == 1 && turninfence == false){ 
        time = millis(); //reset time
        turninfence = true;
    }
    
    if (millis() - time < 7500 && turninfence == true){
    drive();
    }
    if (millis() - time >= 7500 && millis() - time < 8000 && turninfence == true){
    stop();
    }
    if (millis() - time >= 8000 && millis() - time < 15000 && turninfence == true){ 
    turn();
    }
    if (millis() - time >= 15000 && millis() - time < 15500 && turninfence == true){
    stop();
    }
    if (millis() - time >= 15500 && turninfence == true){ 
      resetTimer = false;
      resetTimer2 = false;
      resetTimer3 = false;
      avoidcurrent = false;
      avoidfence = false;
      avoidUH = false;
      turninfence = false;
    }
    
  } //else if out of fence
 
} //move


////////////////////////  geofence  ////////////////////////////////////////
void getFence(){
  int i, j, c = 0;
  for (i = 0, j = npol - 1; i < npol; j = i++){
    if ((((yp[i] <= y) && (y < yp[j])) ||
         ((yp[j] <= y) && (y < yp[i]))) &&
        (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
      c = !c;
  }
  inFence = c;
}//geofence



/////////////////////////////measure current//////////////////////////////////
float measureCurrent(byte pin){  
  while (currentsample_count < CURRENT_SAMPLES)
  {
    currentsum += analogRead(pin);
    currentsample_count++;
    //delay(1);
  }
  ///0.00488 = 5/1024
  float amp = (float)currentsum / (float)CURRENT_SAMPLES * 0.00488 / 0.1;

  currentsample_count = 0;
  currentsum = 0;

  return amp;

}


////////////////////////////  measure voltage  ////////////////////////////////
void measureVoltage(){
  // take a number of analog samples and add them up
  while (voltsample_count < VOLT_SAMPLES)
  {
    voltsum += analogRead(voltPin);
    voltsample_count++;
    //delay(1);
  }
  // calculate the voltage
  // use 5.0 for a 5.0V ADC reference voltage
  // 5.015V is the calibrated reference voltage
  voltage = ((float)voltsum / (float)VOLT_SAMPLES * 5) / 1024.0;
  // send voltage for display on Serial Monitor
  // voltage multiplied by 11 when using voltage divider that
  // divides by 11. 11.132 is the calibrated voltage divide
  // value
  voltage = voltage * 11;
  voltsample_count = 0;
  voltsum = 0;


}
//////////////////////////////////////////////////////////////////////
