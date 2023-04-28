   // #define DEBUG
    #define MAXSPEED 150
    #define CONVEYERSTEP 246//247 - straadaaja labi
    #define TENMETERS 161 //296 //20m // 322-22m //161 - 10m //pulses on encoder
    #define DEBTIME 3
    
    //MOTOR 3 - MAIN FOR MOVING FORWARD OR BACK
    #define M_A1 A5
    #define M_A2 12
    #define M_PA 11

    //SYREN 50 draiverim
    #define S1 11
    #define S2 A5

    //DC driver 
    //MOTOR 1
    #define R1PWM 9
    #define L1PWM 10

    //MOTOR 2
    #define R2PWM 6
    #define L2PWM 5


    #define BUZZ 13

    
    #define END1 3
    #define END2 8

    #define CH1 A0
    #define CH2 A4
    //#define CH3 A2
    #define CH4 A1
    #define CH5 A2
    #define CH6 A3


    //STEPER COMMON
    #define stepsPerRevolution 200*4
    #define speedStepperMotor 1100
    
    //STEPPER 1
    #define STP1DIR 1
    #define STP1PUL 0
    
    //STEPPER 2

    #define STP2DIR 4
    #define STP2PUL 7

    #define ENCODER 2
       
    #define SOUNDON digitalWrite(BUZZ,HIGH); // Send 1KHz sound signal...
    #define SOUNDOFF digitalWrite(BUZZ,LOW);;     // Stop sound...
     
    //FUNCTION DEFINITIOSN
    void mot2DOWN();
    void mot1DOWN();
    void mot2Stop();
    void mot1Stop();
    void mot2UP();
    void mot1UP();
    void mainMotorForward(uint8_t motSpeed);
    void mainMotorBackward(uint8_t motSpeed);
    void mainMotorStop();
    void mainMotorBrake();
    void extendArms();
    void moveConveyer();
    void conveyerForward();
    void conveyerBackward();
    
    long long timePerLoop1 =0, timePerLoop2 = 0, millis_at_last_remote_read = 0, timeToFail = 0;
    uint8_t armDirection = 0; //0 - UP 1 - DOWN
    uint8_t endSW1 = 0,endSW2 =0, movementNotFinished1 = 1, movementNotFinished2 = 1;
    uint8_t swBstate = 0,swCstate = 0, extensionCount =0, tmp1 = 0, flagsPut = 0;
    volatile uint16_t count = 0;
    
    uint16_t ch1 = 0;
    uint16_t ch2 = 0;
    uint16_t ch3 = 0;
    uint16_t ch4 = 0;
    uint16_t ch5 = 0;
    uint16_t ch6 = 0;

void setup() {

   pinMode(END1, INPUT);
   pinMode(END2, INPUT);
   digitalWrite(END1,HIGH);
   digitalWrite(END2,HIGH);

   //ARM MOTORS
   pinMode(R1PWM,OUTPUT);
   pinMode(L1PWM,OUTPUT);
   pinMode(R2PWM,OUTPUT);
   pinMode(L2PWM,OUTPUT);

   //STEPPER MOTORS
   pinMode(STP1DIR,OUTPUT);
   pinMode(STP2DIR,OUTPUT);
   pinMode(STP1PUL,OUTPUT);
   pinMode(STP2PUL,OUTPUT);
   
   pinMode(CH1,INPUT);
   pinMode(CH2,INPUT); 
   //pinMode(CH3,INPUT);
   pinMode(CH4,INPUT); 
   pinMode(CH5,INPUT);
   pinMode(CH6,INPUT); 

   //ARM MOTORS 
   digitalWrite(R1PWM,LOW);
   digitalWrite(L1PWM,LOW);
   digitalWrite(R2PWM,LOW);
   digitalWrite(L2PWM,LOW);        

   //MAIN MOTOR
   pinMode(M_A1,OUTPUT);
   pinMode(M_A2,OUTPUT);
   pinMode(M_PA,OUTPUT);

   //BUZZER
   pinMode(BUZZ,OUTPUT);
   
   //ENCODER
   pinMode(ENCODER, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(ENCODER), updateEncoder, RISING);


#ifdef DEBUG
   Serial.begin(9600);
#endif

   millis_at_last_remote_read = millis();
   count = 0;

   SOUNDON;
   delay(200);
   SOUNDOFF;
   delay(100);
   SOUNDON;
   delay(200);
   SOUNDOFF;
   delay(100);
   SOUNDON;
   delay(200);
   SOUNDOFF;
   delay(100);

    // Serial.print("go home");
}
    
void loop() {
  uint8_t mainMotorSpeed = 0;

  
  if (millis() - millis_at_last_remote_read >= 30)
        {          
          millis_at_last_remote_read = millis();
          ch1 = pulseIn(CH1,HIGH);//coinveyer
          ch2 = pulseIn(CH2,HIGH);//
          ch4 = pulseIn(CH4,HIGH);//THROTLE
          ch5 = pulseIn(CH5,HIGH);//SWB
          ch6 = pulseIn(CH6,HIGH);//SWC

#ifdef DEBUG          
          Serial.print("REMOTE : ");Serial.print(ch1);Serial.print(" ");Serial.print(ch2);Serial.print(" ");Serial.print(ch4);Serial.print(" ");Serial.print(ch5);Serial.print(" ");Serial.println(ch6);
          Serial.print("ENCODER: ");Serial.println(count);
          Serial.print("SWB    : ");Serial.println(swBstate);
          Serial.print("SWC    : ");Serial.println(swCstate);
          Serial.print("END1    : ");Serial.println(digitalRead(END1));
          Serial.print("END2    : ");Serial.println(digitalRead(END2));
#endif    
          //switch B
          if (ch5 < 1500) {swBstate = 0;}
            else if(ch5 > 1500 && ch5 <2100)  {swBstate = 1;}

          //switch C
          if (ch6 < 1300) {swCstate = 0;}
            else if (ch6 > 1700) {swCstate = 2;}
            else {swCstate = 1;}
            
          //load conveyer
          if (ch1 < 1300 && ch1 != 0) {         if (swCstate == 1 && swBstate == 0) {conveyerBackward(); tmp1 = 0;}}
            else if (ch1 > 1655 && ch1 < 2200) {if (swCstate == 1 && swBstate == 0) {conveyerForward();tmp1 = 0;}}
            //else {}
          
          //main motor                            
          if (ch4 < 1470 && ch4 != 0) {               if(ch4<1000){ch4=1000;} mainMotorSpeed = map(ch4,1470,1000,0,MAXSPEED); if (swBstate == 1 && (swCstate == 1 || swCstate == 2)) { mainMotorForward(mainMotorSpeed) ;}}
            else if (ch4 > 1515 && ch4 < 2200) {      if(ch4>1950){ch4=1950;} mainMotorSpeed = map(ch4,1515,1950,0,MAXSPEED); if (swBstate == 1 && (swCstate == 1 || swCstate == 2)) { mainMotorBackward(mainMotorSpeed);}}
            else {mainMotorStop();}

          

                                
        }//END OF if (millis() - millis_at_last_remote_read >= 30){       


        if (swBstate == 0 && swCstate == 0 ) //notiiram svariigos mainiigos
        {
          count = 0;
          mainMotorStop();
          tmp1 = 0;
          flagsPut = 0;
        }
        else if (swBstate == 1 && swCstate == 1 )
        {
          count = 0;
        }
        else if (swBstate == 0 && swCstate == 1 ) //notiiram svariigos mainiigos
        {
          tmp1 = 0;
        }
        else if (swBstate == 1 && swCstate == 0 ) //notiiram svariigos mainiigos
        {
          count = 0;
          mainMotorStop();
          tmp1 = 0;
          flagsPut = 0;
        }
        
        else if (swBstate == 0 && swCstate == 2 ) // TEST ARMS
        {
           if(tmp1<1){  
            unsigned char dont = 0;
            unsigned int ttt =  0;   
            long long timeIN = 0;    
             //mainMotorBrake();
             SOUNDON;
             count = 0;                                    
             moveConveyer();
             //delay(500);
             timeIN = millis(); 
             while(millis() - timeIN < 1000)
               {
                //Serial.print("tt");
                ttt =  pulseIn(CH1,HIGH); 
                if(ttt>1300 && ttt<1655){}
                else {dont = 1;break;}
               }
             if(!dont)
              {extendArms();}
             SOUNDOFF;
             tmp1++;
           } 
        }        
        else if (swBstate == 1 && count > TENMETERS && swCstate == 2) //viens metrs ~ 14 enkodera pulsi 148 == 10 m teorçtiski
          {       
            unsigned char dont = 0;
            unsigned int ttt =  0;   
            long long timeIN = 0; 
                  SOUNDON; 
                  delay(300);               
                  mainMotorStop();
                  delay(300);
                  if(flagsPut>=1){
                    moveConveyer(); 
                  }
                  flagsPut++;
                  count = 0;                  
                  //delay(300);   
                  timeIN = millis(); 
                  while(millis() - timeIN < 1000)
                     {
                      //Serial.print("tt");
                      ttt =  pulseIn(CH1,HIGH); 
                      if(ttt>1300 && ttt<1655){}
                      else {dont = 1;break;}
                     }
                   if(!dont)   {        
                        extendArms();
                    }
                  delay(300);  
                  SOUNDOFF;    
          }
        
  }// END of LOOP

  /**************** FUNCTIONS ***************/
void mot1UP(){
  analogWrite(L1PWM,155);
  analogWrite(R1PWM,0);
}

void mot2UP(){
  analogWrite(L2PWM,150);
  analogWrite(R2PWM,0);
}

void mot1Stop() {
  analogWrite(L1PWM,0);
  analogWrite(R1PWM,0);
}

void mot2Stop() {
  analogWrite(L2PWM,0);
  analogWrite(R2PWM,0);
}

void mot1DOWN(){

  analogWrite(L1PWM,0);
  analogWrite(R1PWM,220);
}

void mot2DOWN(){
  analogWrite(L2PWM,0);
  analogWrite(R2PWM,220);
}

/* //OLD DRIVER
void mainMotorForward(uint8_t motSpeed)
{
  digitalWrite(M_A1,LOW);
  digitalWrite(M_A2,HIGH);
  analogWrite(M_PA, motSpeed);
}

void mainMotorBackward(uint8_t motSpeed)
{
  digitalWrite(M_A1,HIGH);
  digitalWrite(M_A2,LOW);
  analogWrite(M_PA, motSpeed);
}

void mainMotorStop()
{
  digitalWrite(M_A1,HIGH);
  digitalWrite(M_A2,LOW);
  analogWrite(M_PA, 0);
}

void mainMotorBrake()
{
  digitalWrite(M_A1,LOW);
  digitalWrite(M_A2,LOW);
}
*/
//SYREN 50 vadiibai
void mainMotorForward(uint8_t motSpeed)
{
  digitalWrite(S2,LOW);
  analogWrite(S1, motSpeed);
}

void mainMotorBackward(uint8_t motSpeed)
{
  digitalWrite(S2,HIGH);
  analogWrite(S1, motSpeed);
}

void mainMotorStop()
{
  digitalWrite(S2,LOW);
  analogWrite(S1, 0);
}

void extendArms()
{
  movementNotFinished1 = 1;
        movementNotFinished2 = 1;
        endSW1 = 1;
        endSW2 = 1;
#ifdef DEBUG        
        Serial.println("UP");
#endif        
        //GO UP
        timePerLoop1 = millis(); 
        timeToFail = millis();       
        while(movementNotFinished1 || movementNotFinished2){
          //Serial.println("w begin");
          if(movementNotFinished1){mot1UP();}
          if(movementNotFinished2){mot2UP();} 
          if (millis()- timeToFail > 10000) {mot2Stop();mot1Stop();break;}
          if (millis()- timePerLoop1 > 1000)
              {
                //Serial.println("time");
                endSW1 = digitalRead(END1);
                endSW2 = digitalRead(END2);
              }   
#ifdef DEBUG 
                  Serial.print("END1: ");
                  Serial.print(endSW1);
                  Serial.print(" END2: ");
                  Serial.println(endSW2);
#endif
                     
          if(endSW1 == 0){
            delay(DEBTIME);
            endSW1 = digitalRead(END1);
               if(endSW1 == 0){
             // Serial.println("e1");
                mot1Stop();
                movementNotFinished1 = 0;
               }
          }
          if(endSW2 == 0){
             delay(DEBTIME);
             endSW2 = digitalRead(END2);
             if(endSW2 == 0){
            //Serial.println("e2");
              mot2Stop();
              movementNotFinished2 = 0;}
          }
        }//END OF while(movementNotFinished)     
        
        delay (1000);
        
        movementNotFinished1 = 1;
        movementNotFinished2 = 1;
        endSW1 = 1;
        endSW2 = 1;
        timePerLoop1 = millis();
        timeToFail = millis(); 
#ifdef DEBUG        
        Serial.println("DOWN");
#endif
        while(movementNotFinished1 || movementNotFinished2){  
     
          if(movementNotFinished1){mot1DOWN();}
          if(movementNotFinished2){mot2DOWN();} 
          if (millis()- timeToFail > 10000) {mot2Stop();mot1Stop();break;}
          if (millis()- timePerLoop1 > 700)
              { 
                endSW1 = digitalRead(END1);
                endSW2 = digitalRead(END2);
              }         

#ifdef DEBUG 
                  Serial.print("END1: ");
                  Serial.print(endSW1);
                  Serial.print(" END2: ");
                  Serial.println(endSW2);
#endif   
               
          if(endSW1 == 0){
            delay(DEBTIME);
            endSW1 = digitalRead(END1);
            if(endSW1 == 0){
              mot1Stop();
              movementNotFinished1 = 0;}
          }
          if(endSW2 == 0){
            delay(DEBTIME);
            endSW2 = digitalRead(END2);
            if(endSW2 == 0){
              mot2Stop();
              movementNotFinished2 = 0;}
          }
        }//end of while
        movementNotFinished1 = 1;
        movementNotFinished2 = 1;
        endSW1 = 1;
        endSW2 = 1;
}

void moveConveyer()
{
  digitalWrite(STP1DIR, HIGH);
  digitalWrite(STP2DIR, LOW);
  for (int i = 0; i < CONVEYERSTEP; i++) //JANOKALIBREE UZ VIENU KARODZINJU!!!! 260 == 63 mm // 1090 mm viss celsh // 60mm = 249 //223 = 5.5cm
     {
     // These four lines result in 1 step:
     digitalWrite(STP1PUL, HIGH);
     digitalWrite(STP2PUL, HIGH);
     delayMicroseconds(speedStepperMotor);
     digitalWrite(STP1PUL, LOW);
     digitalWrite(STP2PUL, LOW);
     delayMicroseconds(speedStepperMotor);
     } 
}

void conveyerForward()
{
  digitalWrite(STP1DIR, HIGH);
  digitalWrite(STP2DIR, LOW);
  for (int i = 0; i < CONVEYERSTEP; i++) //JANOKALIBREE UZ VIENU KARODZINJU!!!!
     {
     // These four lines result in 1 step:
     digitalWrite(STP1PUL, HIGH);
     digitalWrite(STP2PUL, HIGH);
     delayMicroseconds(speedStepperMotor);
     digitalWrite(STP1PUL, LOW);
     digitalWrite(STP2PUL, LOW);
     delayMicroseconds(speedStepperMotor);
     } 
}

void conveyerBackward()
{
  digitalWrite(STP1DIR, LOW);
  digitalWrite(STP2DIR, HIGH);
  for (int i = 0; i < CONVEYERSTEP; i++) //JANOKALIBREE UZ VIENU KARODZINJU!!!!
     {
     // These four lines result in 1 step:
     digitalWrite(STP1PUL, HIGH);
     digitalWrite(STP2PUL, HIGH);
     delayMicroseconds(speedStepperMotor);
     digitalWrite(STP1PUL, LOW);
     digitalWrite(STP2PUL, LOW);
     delayMicroseconds(speedStepperMotor);
     } 
}

void updateEncoder()
{
  count++;
}
