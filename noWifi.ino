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
unsigned long amptime = millis();
unsigned long time = millis();
unsigned long mowertime = millis();
unsigned long gotime = millis();
unsigned long gpsblink = millis();
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
////////////////////////geofence////////////////////////////////////////


//////////////////////// WHEEL MOTORS /////////////////////////////////////
bool avoid = false;
bool dirchange = false;
int direction = 1;

///////////////////////////////////////////////////////////////////////////////



/////////////////////////////// UH  ///////////////////////////////////////
#include <Ultrasonic.h>
Ultrasonic ultrasonic(UH1, UH2);
int distance;

/////////////////////////////////Current//////////////////////////////////
#define CURRENT_SAMPLES 10
int currentsum = 0;                    // sum of samples taken
int currentsample_count = 0; // current sample number
float current = 0.0;
float current2 = 0.0;
bool highcurrent = false;
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
















void setup() {
 
  
  
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
  //pinMode(A1, INPUT);
  pinMode(voltPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(currentPin, INPUT);
  pinMode(currentPin2, INPUT);
  
}










bool start = false;
bool cutstart = false;
void loop()
{

  
  addFence();

  //measureVoltage();      //get voltage from voltage divider r1=1Mohm, r2=100Kohm, takes 10ns
  //Serial.print(voltage);
  //Serial.println (" V");
  //low voltage
  /* 
    if (voltage <= 11.8) {
     
    }
  */

  //measureCurrent(A1);       //get current from 0.1ohm shunt. takes 10ns
  current = measureCurrent(currentPin);       //get current from 0.1ohm shunt. takes 10ns
  current2 = measureCurrent(currentPin2);       //get current from 0.1ohm shunt. takes 10ns

  //Serial.println(current);
  //Serial.println(current2);

  
 
  
  if (startbutton.onPressed()){
    gotime = millis();
    //digitalWrite(LED_BUILTIN, HIGH);
    start= true;
    cutstart=true;
  }
  getCoords();
  getDistance(); 
  
  //start wheels
  if (millis() - gotime > 10000 && start==true){  
    moveRobot();         
  }
  //start cutter
  if (start == true && cutstart == true){
    digitalWrite(cutter, HIGH);
    cutstart= false;
    mowertime = millis();
  }

    //check if cutter jammed
    if (current2 > 2 && millis()-mowertime > 2000){               
        digitalWrite(cutter, LOW);
        mowertime = millis();
        
    }
    
    if (digitalRead(cutter) == LOW && millis() - mowertime > 5000 && start == true){
      digitalWrite(cutter, HIGH);
      mowertime = millis();
      digitalWrite(ledPin, LOW);
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
    //fence
    getFence();         //kasuta getCoords geofence arvutamiseks    
    //Serial.println(inFence);
    //fence
    //DEBUG_PORT.print( F("Location: ") );
    if (fix.valid.location)
    {
      digitalWrite(ledPin, HIGH);
      y = (fix.latitudeL()); // integer displayco
      //Blynk.virtualWrite(V0, y);
      //DEBUG_PORT.print(y);
      //DEBUG_PORT.print( ',' );
      x = (fix.longitudeL()); // integer display
      //Blynk.virtualWrite(V1, x);
      //DEBUG_PORT.println(x);
    }
    if(millis() - gpsblink > 500){
      digitalWrite(ledPin, LOW);
    }
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
  digitalWrite(fwdright, HIGH); //turn
  digitalWrite(revright, LOW);
  digitalWrite(revleft, HIGH);
  digitalWrite(fwdleft, LOW);
}

void testcurrent (){
  if (current > 1 && highcurrent == false){
    amptime = millis(); 
    highcurrent = true;  
  }

  if (current > 1 && millis() - amptime > 1000 && highcurrent == true){
      avoidcurrent = true;
  }
}//test current



void moveRobot(){
  
  testcurrent();
  //Serial.println(current);

//all ok movement
  if (distance > 19 && avoid == false  && inFence == 1  &&  avoidcurrent == false) {
    direction = 1;
    drive();
  }

// fence trigger
 
  else if (inFence == 0 && avoidcurrent == false){
    if (avoid == false){
    avoid = true;
    time = millis(); //and reset time
    }
    
    if (millis() - time < 500)
    {
    stop();
    }
    if (millis() - time > 500)
    { //Has 0.5 second passed?
      direction = -1;
      drive();
      testcurrent();
    }
    if (inFence == 1)
    { 
      avoid = false;
    }
  } //if out of fence


///UH trigger
  else if (distance <= 19  && inFence == 1  &&  avoidcurrent == false){
    if (avoid == false){
    avoid = true;
    time = millis(); //and reset time
    }
    
    if (millis() - time < 500)
    {
    stop();
    }
    if (millis() - time > 500 &&  millis() - time < 4500) //back up 4 sec
    { //Has 0.5 second passed?
      direction = -1;
      drive();
      testcurrent();
    }
    if (millis() - time > 4500 && millis() - time < 5000)
    { //Has 2 second passed?
      stop();
    }
    if (millis() - time > 5000 && millis() - time < 9000)
    { //Has 0.5 second passed?
      turn();
      testcurrent();
    }
    if (millis() - time > 9000 && millis() - time < 9500)
    { //Has 2 second passed?
      stop();
    }

    if (millis() - time > 9500)
    { //Has 2 second passed?
      avoid = false;
    }
  } //if UH trigger

// current trigger
else if (avoidcurrent == true){
    if (avoid == false){
    avoid = true;
    time = millis(); //and reset time
    }
    
    if (millis() - time < 500)
    {
    stop();
    }
    if (millis() - time > 500 &&  millis() - time < 4500) // change dir 4 4 sec
    { //Has 0.5 second passed?
    if (dirchange == false){
      direction *= -1;
      dirchange = true;
      }
      drive();
      testcurrent();
    }
    if (millis() - time > 4500 && millis() - time < 5000)
    { //Has 2 second passed?
      dirchange = false;
      stop();
    }
    if (millis() - time > 5000 && millis() - time < 9000)
    { //Has 0.5 second passed?
      turn();
      testcurrent();
    }
    if (millis() - time > 9000 && millis() - time < 9500)
    { //Has 2 second passed?
      stop();
    }

    if (millis() - time > 9500)
    { //Has 2 second passed?
      avoid = false;
      avoidcurrent = false;
      highcurrent = false;
    }
  } //if current trigger

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
