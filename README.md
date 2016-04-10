# ArduinoCar
controlled a remote controlled car :car: using an arduino microcontroller

	#include <NewPing.h>

//L298P motor driver Connection   
  const int motorA_DIR  = 12;  
  const int motorA_PWM  = 3;  
  const int motorA_BRK = 9;
  const int motorB_DIR  = 13;  
  const int motorB_PWM  = 11;  
  const int motorB_BRK = 8;
  
//Leds connected to Arduino UNO Pin 6

  const int lights  = 6; 
  
//Bluetooth (HC-06 JY-MCU) State pin on pin 2 of Arduino

  const int BTState = 2;
  
//Calculate Battery 
	// Change value to your max battery voltage level! 
	
  const float maxBattery = 8.0;
  
  // Percentage variable
  
  int perVolt;                 
  
  // Read battery voltage
  
  float voltage = 0.0;         
  
  int level;
// Use it to make a delay... without delay() function!
	// -1000*10=-10sec. to read the first value. If you use 0 then you will take the first value after 10sec.  
  long previousMillis = -1000*10;
  // interval at which to read battery voltage, change it if you want! (10*1000=10sec)
  long interval = 1000*10;       
  //unsigned long currentMillis;
  unsigned long currentMillis;   
//Useful Variables
  int i=0;
  int j=0;
  int state;
  // Default speed, from 0 to 255
  int vSpeed=200;     
  int obstacle_distance = 0;
  
//Setting up Ultra Sonic Sensor
 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define TRIGGER_PIN  7
  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN     4
// Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MAX_DISTANCE 200 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

  // Set pins as outputs:
   void setup() {
    pinMode(motorA_DIR, OUTPUT);
    pinMode(motorA_PWM, OUTPUT);   
    pinMode(motorA_BRK, OUTPUT);
    pinMode(motorB_DIR, OUTPUT);
    pinMode(motorB_PWM, OUTPUT);
    pinMode(motorB_BRK, OUTPUT);
    pinMode(lights, OUTPUT); 
    pinMode(BTState, INPUT);    
    // Initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}
  //Stop car when connection lost or bluetooth disconnected 
  
  void loop() {
  	obstacle	_distance = sonar.ping_cm();
  	if(digitalRead(BTState)==LOW) {
  		state='S';
  		}

	//  Save income data to variable 'state'

    if(Serial.available() > 0) {
    	state = Serial.read();
    	}

  //Stop car when meets an obstacle
  //Change speed if state is equal from 0 to 4. Values must be from 0 to 255 (PWM)

    if(obstacle_distance>50) {
    	state='S';
    	}
    else if (state == '0'){
    	vSpeed=0;
    }
    else if (state == '1'){
    	vSpeed=100;
    }
    else if (state == '2'){
    	vSpeed=180;
    }
    else if (state == '3'){
    	vSpeed=200;
    }
    else if (state == '4'){
    	vSpeed=255;
    }
 	  
  /***********************Forward****************************/
  
  //If state is equal with letter 'F', car will go forward!

    if (state == 'F') {
        digitalWrite(motorA_DIR,HIGH);
        digitalWrite(motorB_DIR,HIGH);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,vSpeed);
        analogWrite(motorB_PWM,vSpeed);
    }
  /**********************Forward Left************************/
  
  //If state is equal with letter 'G', car will go forward left

    else if (state == 'G') {
        digitalWrite(motorA_DIR,HIGH);
        digitalWrite(motorB_DIR,HIGH);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,vSpeed);
        analogWrite(motorB_PWM,max(vSpeed+20,255));
    }
  /**********************Forward Right************************/
  
  //If state is equal with letter 'I', car will go forward right

    else if (state == 'I') {
      	digitalWrite(motorA_DIR,HIGH);
        digitalWrite(motorB_DIR,HIGH);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,max(vSpeed+20,255));
        analogWrite(motorB_PWM,vSpeed);
    }
  /***********************Backward****************************/
  
  //If state is equal with letter 'B', car will go backward

    else if (state == 'B') {
    	  digitalWrite(motorA_DIR,LOW);
        digitalWrite(motorB_DIR,LOW);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,vSpeed);
        analogWrite(motorB_PWM,vSpeed);
    }
  /**********************Backward Left************************/
  
  //If state is equal with letter 'H', car will go backward left

    else if (state == 'H') {
    	  digitalWrite(motorA_DIR,LOW);
        digitalWrite(motorB_DIR,LOW);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,vSpeed);
        analogWrite(motorB_PWM,max(vSpeed+20,255));
    }
  /**********************Backward Right************************/
  
  //If state is equal with letter 'J', car will go backward right

    else if (state == 'J') {
    	  digitalWrite(motorA_DIR,LOW);
        digitalWrite(motorB_DIR,LOW);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,max(vSpeed+20,255));
        analogWrite(motorB_PWM,vSpeed);
    }
  /***************************Left*****************************/
  
  //If state is equal with letter 'L', wheels will turn left

    else if (state == 'L') {
    	  digitalWrite(motorA_DIR,LOW);
        digitalWrite(motorB_DIR,HIGH);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,vSpeed);
        analogWrite(motorB_PWM,vSpeed);  
    }
  /***************************Right*****************************/
  
  //If state is equal with letter 'R', wheels will turn right

    else if (state == 'R') {
        digitalWrite(motorA_DIR,HIGH);
        digitalWrite(motorB_DIR,LOW);
        digitalWrite(motorA_BRK,LOW);
        digitalWrite(motorB_BRK,LOW);
        analogWrite(motorA_PWM,vSpeed);
        analogWrite(motorB_PWM,vSpeed);
    }
  /************************Lights*****************************/
  
  //If state is equal with letter 'W', turn leds on or of off

    else if (state == 'W') {
      if (i==0){  
         digitalWrite(lights, HIGH); 
         i=1;
      }
      else if (i==1){
         digitalWrite(lights, LOW); 
         i=0;
      }
      state='n';
    }
  /************************Stop*****************************/
  
  //If state is equal with letter 'S', stop the car

    else if (state == 'S'){
        digitalWrite(motorA_DIR,HIGH);
        digitalWrite(motorB_DIR,HIGH);
        digitalWrite(motorA_BRK,HIGH);
        digitalWrite(motorB_BRK,HIGH);
        analogWrite(motorA_PWM,0);
        analogWrite(motorB_PWM,0);
    }
    
  /***********************Battery*****************************/
  
  //Read battery voltage every 10sec.

    currentMillis = millis();
    if(currentMillis - (previousMillis) > (interval)) {
       previousMillis = currentMillis; 
       //Read voltage from analog pin A0 and make calibration:
       voltage = (analogRead(A0)*5.015 / 1024.0)*11.132;
       //Calculate percentage...
       perVolt = (voltage*100)/ maxBattery;
       if      (perVolt<=75)               { level=0; }
       else if (perVolt>75 && perVolt<=80) { level=1; }    //        Battery level
       else if (perVolt>80 && perVolt<=85) { level=2; }    //Min ------------------------   Max
       else if (perVolt>85 && perVolt<=90) { level=3; }    //    | 0 | 1 | 2 | 3 | 4 | 5 | >
       else if (perVolt>90 && perVolt<=95) { level=4; }    //    ------------------------
       else if (perVolt>95)                { level=5; }   
       Serial.println(level);    
    }
    
}


