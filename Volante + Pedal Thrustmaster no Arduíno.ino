 * Enhancements: 
 *  - supports hot swapping of Thrustmaster R383, F1 and Ferrari 599xx wheels
 *  - all buttons are working
 *  - button numbering is the same as when the wheels are on a Thrustmaster base
 *    (paddles are always buttons 1 and 2 for example)
 *    
 */

/*
* On Arduino Pro Micro side it must be connected as follows:
 *          ----___----
 *        / 6   [ ]   5 \
 *       |      [_]      |   (as seen from the female front of the connector)
 *       | 4           3 |
 *        \___       ___/
 *            |2   1|
 *            \_____/
 *  
 base (femea)          
 5 = vermelho  = vermelho
 1 = preto     = verde orig
 2 = azul  =   azul
 3 = laranja/branco = branc
 6 = roxo/verde   =  cinzaz org
 4 =  amarelo  = laranj orig

 rosca
 2 = preto      (1)
 1 = azul       (2)
 4 = branco     (3)
 3 = amarelo    (4)
 6 = vermelho   (5)
 5 = verde      (6)

 * 1              -> not used (or can be connected to arduino MOSI pin 16) (Blue)
 * 2 Green - GND  -> Arduino Pro Micro GND pin (Gray)
 * 3 White - MISO -> Arduino Pro Micro pin 14 (white/orange)
 * 4 Yellow - SS  -> Arduino Pro Micro pin 7 (Yellow)
 * 5 Black - SCK  -> Arduino Pro Micro pin 15 (Black)
 * 6 Red - +VCC   -> Arduino +5V pin (or RAW if USB current is +5V already) (Red)
 * 
 * Wheels and official Thrustmaster button numbers:
 * 
 * R383 wheel                     F1 wheel                       Ferrari 599xx
 * ----------------------------   ----------------------------   --------------------------------
 * Byte 1                         Byte 1                         Byte 1
 * 7 - constant 0                 7 - DRS (5)    (yes, odd)      7 - constant 1
 * 6 - constant 0                 6 - constant 0                 6 - constant 0
 * 5 - constant 1                 5 - constant 1                 5 - constant 1
 * 4 - constant 1                 4 - constant 1                 4 - left paddle (1)
 * 3 - constant 0                 3 - left paddle (1)            3 - right paddle (2)
 * 2 - constant 1                 2 - right paddle (2)           2 - PIT (blue upper left) (3)
 * 1 - constant 0                 1 - N (3)                      1 - WASH (blue down left) (4)
 * 0 - bottom right yellow (6)    0 - PIT (4)                    0 - RADIO (blue upper right) (5)
 * 
 * Byte 2                         Byte 2                         Byte 2
 * 7 - right paddle (2)           7 - START (13)                 7 - black down left (6)
 * 6 - top right black (5)        6 - 10+ (6)                    6 - MAIN left (7)
 * 5 - top right red (9)          5 - B0 (7)                     5 - MAIN right (8)
 * 4 - constant 0                 4 - WET (8)                    4 - SCROLL (red upper right) (9)
 * 3 - bottom left red (7)        3 - PL (9)                     3 - FLASH (red upper left) (10)
 * 2 - bottom right white (13)    2 - K (10)                     2 - constant 0
 * 1 - bottom right red (8)       1 - PUMP (11)                  1 - constant -
 * 0 - constant 0                 0 - 1- (12)                    0 - MAIN pushed in (13)
 * 
 * Byte 3                         Byte 3                         Byte 3
 * 7 - bottom left yellow (3)     7 - right Dpad up (18)         7 - Dpad down
 * 6 - top left red (10)          6 - left Dpad down             6 - Dpad right
 * 5 - Dpad left                  5 - left Dpad right            5 - Dpad left
 * 4 - Dpad up                    4 - left Dpad left             4 - Dpad up
 * 3 - Dpad right                 3 - left Dpad up               3 - constant 0
 * 2 - Dpad down                  2 - right Dpad down (19)       2 - constant 0
 * 1 - bottom left yellow (4)     1 - right Dpad right (20)      1 - constant 0
 * 0 - left paddle                0 - right Dpad left (21)       0 - constant 0
 * 
 * Byte 4                         Byte 4
 * 7 - 1 when Dpad pressed        7 - 
 * 6 - 1 when Dpad pressed        6 - CHRG down (16)
 * 5 - 1 when Dpad pressed        5 - DIF IN up (14)
 * 4 - 1 when Dpad pressed        4 - DIF IN down (15)
 * 3 - 1 when Dpad pressed        3 - CHRG up (17)
 * 2 - 1 when Dpad pressed        2 - 
 * 1 - 1 when Dpad pressed        1 - 
 * 0 - 1 when Dpad pressed        0 -
*/

#include <SPI.h>
#include <Joystick.h>

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, 21, 1,    // 21 buttons, 1 hatswitch
  false, false, true, true, true, true,
  false, true, false, false, false);  
  
const int slaveSelectPin = 7;
byte pos[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
byte currBytes[] = {0x00, 0x00, 0x00, 0x00, 0x00};
byte prevBytes[] = {0x00, 0x00, 0x00, 0x00, 0x00};
int wheelbyte, fourthbyte, fifthbyte, wheelID;
bool btnState, joyBtnState, prevJoyBtnState, buttonsreset, wheelIdentified;
const bool debugging = false; // Set to true to see wheel bits debug messages on the com port
int bit2btn[] = {-1,-1,-1,-1,-1,-1,-1,-1,  -1,-1,-1,-1,-1,-1,-1,-1,  -1,-1,-1,-1,-1,-1,-1,-1,  -1,-1,-1,-1,-1,-1,-1,-1}; // working array of buttons
int F599Btn[] = {-1,-1,-1,0,1,2,3,4,  5,6,7,8,9,-1,-1,12,  33,32,34,31,-1,-1,-1,-1,  -1,-1,-1,-1,-1,-1,-1,-1}; // button numbers 599xx wheel
int R383Btn[] = {-1,-1,-1,-1,-1,-1,-1,5,  1,4,8,-1,6,12,7,-1,  2,9,34,31,32,33,3,0,  -1,-1,-1,-1,-1,-1,-1,-1}; // button numbers R383 wheel
int F1Btn[] = {4,-1,-1,-1,0,1,2,3,  12,5,6,7,8,9,10,11,  17,33,32,34,31,18,19,20,  -1,15,13,14,16,-1,-1,-1}; // button numbers F1 wheel

int zAxis_ = 0; 
int RxAxis_ = 0;                    
int RyAxis_ = 0;  
int RzAxis_ = 0;          
int Throttle_ = 0;         

const bool initAutoSendState = true; 

  //capacitor emulation - evitar oscilação do ruído eletrico
  float alphaC = 0.8;  // Fator de suavização (0 < alpha < 1)
  float alpha = 0.4;  // Fator de suavização (0 < alpha < 1)
  float smoothedValueT = 0;
  float smoothedValueB = 0;
  float smoothedValueC = 0;


void setup() {
  //input from wheel
  Serial.begin(9600);
  SPCR |= _BV(CPHA);
  SPCR |= _BV(CPOL);
  SPI.beginTransaction(SPISettings(40000, MSBFIRST, SPI_MODE0));
  SPI.begin();                     
  pinMode(slaveSelectPin, OUTPUT);

  //output to joystick
  Joystick.begin();
}

// print byte as binary, zero padded if needed 
// "127" -> "01111111"
void printBinary(byte data) {
 for(int i=7; i>0; i--) {
   if (data >> i == 0) {
     Serial.print("0");
   } else {
     break;
   }
 }
 Serial.print(data,BIN);
 Serial.print(" ");
}

void loop() {


  //Begin T3PA
  // Start setting the millis to no mess with delay of buttons
  unsigned long currentMillis = millis();
  static unsigned long lastPedalReadTime = 0;
  const unsigned long pedalInterval = 50; // Desired interval between pedal readings (in milliseconds)

  if (currentMillis - lastPedalReadTime >= pedalInterval) {
    // Pedal reading and mapping logic

//capacitor emulator
    zAxis_ = analogRead(A0);
    smoothedValueC = alphaC * zAxis_ + (1 - alphaC) * smoothedValueC;
    zAxis_ = map(smoothedValueC, 180, 1920, 0, 512);
    Joystick.setZAxis(zAxis_);

//capacitor emulator
    RxAxis_ = analogRead(A1);
    smoothedValueB = alpha * RxAxis_ + (1 - alpha) * smoothedValueB;
    RxAxis_ = map(smoothedValueB, 180, 1920, 0, 512);
    Joystick.setRxAxis(RxAxis_);

//    RyAxis_ = analogRead(A2);
//    RyAxis_ = map(RyAxis_, 150, 1920, 0, 512);
//    Joystick.setRyAxis(RyAxis_);

//    RzAxis_ = analogRead(A3);
//    RzAxis_ = map(RzAxis_, 1920, 150, 512, 0);
//    Joystick.setRzAxis(RzAxis_);

    //old original
    //Throttle_ = analogRead(A2);
    //Throttle_ = map(Throttle_, 256, 2800, 0, 1024);
    //Joystick.setThrottle(Throttle_);

    //capacitor emulator
    Throttle_ = analogRead(A2);
    smoothedValueT = alpha * Throttle_ + (1 - alpha) * smoothedValueT;
    Throttle_ = map(smoothedValueT, 256, 2800, 0, 1024);
    Joystick.setThrottle(Throttle_);    

    lastPedalReadTime = currentMillis;  // Updates the last pedal reading time
  }

//End T3PA







//start steering wheel buttons

  // tell the wheel, that we are ready to read the data now
  digitalWrite(slaveSelectPin, LOW);
  // the chips in the wheel need some time to wake up
  delayMicroseconds(40);
  
  //read the wheel's 5 bytes
  for(int i=0; i<5; i++) {
    currBytes[i] = ~SPI.transfer(0x00);
    delayMicroseconds(40);
  }

  // release the wheel
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(40);
  
  if (debugging) {
    for(int i=0; i<5; i++) {
      printBinary(currBytes[i]);
      Serial.print("\t");
    }
    Serial.println();
  }

  // Check for sane input: is the wheel plugged in?
  // Unplugged: first byte has all bits set or unset
  // When plugged in F1 and Ferrari 599xx wheel has bits 7, 6, 5 set as 101 (160 dec)
  // Sparco R383 has bits 7, 6, 5 set as 001 (32 dec)
  wheelbyte = currBytes[0] & B11100000;
  fourthbyte = currBytes[3] & B00100000;
  fifthbyte = currBytes[4] & B00001111;
  buttonsreset = false;
  
  while ((wheelbyte != 192) and (wheelbyte != 160) and (wheelbyte != 32)) {  // unknown wheel or wheel unplugged

     wheelIdentified = false;
        
     // Reset all buttons to avoid stuck buttons when unplugged
     if (buttonsreset == false) {
        Joystick.setHatSwitch(0, JOYSTICK_HATSWITCH_RELEASE);  // release hatswitch
        for(int b=0; b<21; b++)        // one button at a time
          Joystick.setButton(b, 0);    // release the button
        buttonsreset = true;           // do it just once
     }
     
     if (debugging) Serial.println("Wheel not plugged in, waiting...");

     // tell the wheel, that we are ready to read the data now
     digitalWrite(slaveSelectPin, LOW);
     // the chips in the wheel need some time to wake up
     delayMicroseconds(40); 
     
     // read the wheel's 5 bytes
     for(int i=0; i<5; i++) {
       currBytes[i] = ~SPI.transfer(0x00);
       delayMicroseconds(40);
     }

     // release the wheel
     digitalWrite(slaveSelectPin, HIGH);
     delayMicroseconds(40);

     if (debugging) for(int i=0; i<5; i++) {
       printBinary(currBytes[i]);
       Serial.print("\t");
     }
     if (debugging) Serial.println();

     wheelbyte = currBytes[0] & B11100000;  // same as above, for identifying the wheel below
     fourthbyte = currBytes[3] & B00100000;
     fifthbyte = currBytes[4] & B00001111;
     
     // Still no sane input? Wait...
     if ((wheelbyte != 192) and (wheelbyte != 160) and (wheelbyte != 32)) {
       //delay(1000);
     }
  }

  if (wheelIdentified == false) {
    if ((wheelbyte == 160) and (fifthbyte == 0)) {
    wheelID = 2;                                           // Ferrari 599xx wheel
    memcpy(bit2btn,F599Btn,sizeof(F599Btn));               // button numbers 599xx wheel to working array
    wheelIdentified = true;
    } else if (wheelbyte == 32) {
             if (((fourthbyte == 0) and (fifthbyte == 0)) or ((currBytes[3] == 255) and (currBytes[4] == 255))) {
                wheelID = 3;                               // Sparco R383 connected
                memcpy(bit2btn,R383Btn,sizeof(R383Btn));   // button numbers R383 wheel to working array
                wheelIdentified = true;
             } else if ((wheelbyte == 32) and (fifthbyte == 15)) {      // F1 wheel sets last four bits of byte 5 as 1, 599xx wheel sets byte 5 as zero
                      wheelID = 1;                          // F1 wheel connected
                      memcpy(bit2btn,F1Btn,sizeof(F1Btn));  // button numbers F1 wheel to working array
                      wheelIdentified = true;
                    }
           }
  }

  if (debugging) { 
    Serial.print(wheelbyte);
    Serial.print("\t");
    Serial.println(wheelID);
  }
  
  // deal with the buttons first
  if (wheelIdentified) 
    for(int i=0; i<4; i++)      //process the four bytes
      for(int b=0; b<8; b++)     //one bit at a time
        if((currBytes[i] & pos[b])!=(prevBytes[i] & pos[b])) {  // if the buttonstate has changed
          btnState=currBytes[i] & pos[b];        
          if ((bit2btn[(i*8)+b] >= 31) and (bit2btn[(i*8)+b] <= 34)) {   // hatswitch (Dpad) pressed?
            if (btnState == 0)               // button released?
              Joystick.setHatSwitch(0, JOYSTICK_HATSWITCH_RELEASE);  // release hatswitch
              else Joystick.setHatSwitch(0, (bit2btn[(i*8)+b] - 31) * 90);  // direction in 0, 90, 180, 270 degrees
          } else Joystick.setButton(bit2btn[(i*8)+b], btnState);      // send the update     
      
      }

  if ((wheelIdentified) and (wheelID == 3)) {                         // only for R383 wheel
    joyBtnState = (currBytes[3] & pos[0]) && !(currBytes[2] & 0x3c);  // if hatswitch is pressed in the middle
    if (joyBtnState != prevJoyBtnState)
      Joystick.setButton(13, joyBtnState);                            // press button 14
  }
  
  for(int i=0;i<5;i++)
    prevBytes[i] = currBytes[i];   // finally update the just read input to the the previous input for the next cycle

  prevJoyBtnState = joyBtnState;


               
 // End Wheel Code



}
