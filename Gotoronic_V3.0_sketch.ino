//Copyright (c) 2016 goloveski
//Released under the MIT license
//http://opensource.org/licenses/mit-license.php
// SERVO REAR
// Red = +6v
// Brown = GND
// Orange = Signal (pin 9 for this code)

// SERVO FRONT
// Red = +6v
// Brown = GND
// Orange = Signal (pin 8 for this code)

#include <EEPROM.h>
#include <Servo.h>
#include <Bounce2.h>

//////////////////////////////// MCU PIN ASSIGN START ///////////////////////////////////////////
#define BUTTON_PIN 2  //REAR
#define BUTTON_PIN2 3 //REAR
#define BUTTON_PIN_f1 4  //FRONT
#define BUTTON_PIN_f2 5  //FRONT
#define SERVO_PIN_F 8  // SERVO F
#define SERVO_PIN_R 9  // SERVO R

#define LED_PIN_MODSEL 15         // LED goes on when Fine Tune Setting mode (NANO)
#define LED_PIN2 13               // LED goes on when pushing shift botton (NANO)
#define BUTTON_PIN_MODESEL 14     //MODE SELECT (NANO)
#define BUTTON_PIN_MODESEL2 16     //MODE SELECT (NANO)

////////////////////////////////  MCU PIN ASSIGN END ///////////////////////////////////////////

//////////////////////////////// EEPROM PARAMETER START ///////////////////////////////////////////
int pos_final_addres = 10; //final gear position save address
int pos_final; //final gear position
int pos_final_addres_f = 9; //final gear position save address front
int pos_final_f; //final gear position front
int pos_memory_addres = 11; //offsetmemory start address 11 12 13 14 15 16 17 18 19 20 21
int pos_f_memory_addres = 31; //offsetmemory front start address 31 32 33 34 35 36 37 38 39 40 41
int pos_memory_addres_zero = 50; //pos_zero_offset_memory address 50
int pos_memory_addres_step = 52; //pos_step_offset_memory address 52
int pos_f_set_memory_addres = 54; //pos_f_offset_memory address 54 55
//////////////////////////////// EEPROM PARAMETER END ///////////////////////////////////////////

//////////////////////////////// REAR SHIFT SETTING PARAMETER START ///////////////////////////////////////////
int speed_step = 9;            //8=9速 9=10速 10=11速
int pos_zero_org = 1820;           // shift 0 position : about 2000~1800
int pos_zero;
int pos_shiftstep_org = 65;        // shift step 56->68 @160404
int pos_shiftstep;
int pos_data_ORG[11] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 11speed offset data
int fine_tune_pitch = 1;      //fine tune pitch data for setting mode
int zero_tune_pitch = 5;      //zero tune pitch data for setting2 mode

int pos_shiftover_up = 15;   // shift over step
int pos_shiftover_down = 10; // shift over step
int pos_shiftover_up_tune = 20; // shift over step (ややUP気味に戻してやる）
int ovsft_delay = 2000;       // over shift delay [msec]
int ovsft_delay2 = 500;       // over shift delay [msec]
int sft_delay = 20;          // shift delay per gear pos [msec]

int wait_msec1 = 1000;     //多段変則設定時間1[msec]
int wait_msec2 = 1200;     //多段変則設定時間2[msec]
int wait_msec3 = 1400;     //多段変則設定時間3[msec]

////////do not change//////////
int pos_data[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //  "do not change" 11speed offset data MATRIX
int pos_data_E2PR[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //  "do not change" 11speed fine tuninng data MATRIX
int pos_data_zero_E2PR; //  "do not change" pos_zero offset
int pos_data_step_E2PR; //  "do not change" pos_step
int pos_old;           // gear position
int pos_new;           // gear position
int pos_direction  = 1;     // shift direction
int pos = 0;    // variable to store the servo position
//////////////////////////////// REAR SHIFT SETTING PARAMETER END ///////////////////////////////////////////

//////////////////////////////// FRONT SHIFT SETTING PARAMETER START  ///////////////////////////////////////////
int pos_front[2] = {1420, 1800};   //{inner, outer}
int pos_shiftover_up_f = -130;   // shift over degree 15->20 @150809
int pos_shiftover_down_f = -50; // shift over degree 15->40 @150809
int pos_data_f_ORG[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 11speed front offset data

////////do not change//////////
int pos_data_f[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //  "do not change" 11speed front offset data MATRIX
int pos_data_f_E2PR[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //  "do not change" 11speed front fine tuninng data MATRIX
int pos_front_E2PR[2]; //  "do not change" front offset
int pos_f = 0;  // variable to store the servo position
//////////////////////////////// FONT SHIFT SETTING PARAMETER END ///////////////////////////////////////////

//////////////////////////////// OTHER PARAMETERS START ///////////////////////////////////////////

// Instantiate a Bounce object
Bounce debouncer  = Bounce();
Bounce debouncer2 = Bounce();
Bounce debouncer3 = Bounce();
Bounce debouncer4 = Bounce();
Bounce debouncerX = Bounce();
Bounce debouncerY = Bounce();

// The current buttonState
// 0 : released
// 1 : pressed less   than 1 seconds
// 2 : pressed longer than 1 seconds
// 3 : pressed longer than 2 seconds
int buttonState;
int buttonState2;
int buttonState3;
int buttonState4;
unsigned long buttonPressTimeStamp;
unsigned long buttonPressTimeStamp2;
unsigned long buttonPressTimeStamp3;
unsigned long buttonPressTimeStamp4;
unsigned long pos_ctrl_end_TimeStamp;

volatile boolean CAL_STAT = false; // fine tune mode
volatile boolean CAL_STAT2 = false; // fine tune mode2

Servo myservo;  // create servo object to control a servo
Servo myservo_f; // a maximum of eight servo objects can be created

char input[2];  //シリアル通信の文字列格納用
char str[0];    //シリアル通信の文字列格納用

//////////////////////////////// OTHER PARAMETERS END ///////////////////////////////////////////


void setup()
{
  pos_final = EEPROM.read(pos_final_addres);
  if (pos_final <  0         ) {
    pos_final = 0; //ギヤポジション0より小を禁止
  }
  if (pos_final >  speed_step) {
    pos_final = 0; //ギヤポジションMAXより大を禁止
  }
  pos_new = pos_final;

  for (int n = 0 ; n <= speed_step ; n++)   //fine tune dataの読み出し
  {
    pos_data_E2PR[n] = EEPROM.read(pos_memory_addres + n );
    if ( pos_data_E2PR[n] >  128) {
      pos_data_E2PR[n] =  pos_data_E2PR[n] - 256; //offset data EEPROMは0～255なので負数はオフセットさせる。
    }
    pos_data[n] =  pos_data_ORG[n] + pos_data_E2PR[n];
    delay(2);
  }
  delay(2);

  pos_data_step_E2PR = EEPROM.read(pos_memory_addres_step);
  if (pos_data_step_E2PR >  128) {
    pos_data_step_E2PR =  pos_data_step_E2PR - 256; //offset data EEPROMは0～255なので負数はオフセットさせる。
  }
  pos_shiftstep = pos_shiftstep_org + pos_data_step_E2PR;
  delay(2);

  pos_data_zero_E2PR = EEPROM.read(pos_memory_addres_zero);
  if (pos_data_zero_E2PR >  128) {
    pos_data_zero_E2PR =  pos_data_zero_E2PR - 256; //offset data EEPROMは0～255なので負数はオフセットさせる。
  }
  pos_zero =  pos_zero_org + pos_data_zero_E2PR;
  delay(2);

  pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new];
  myservo.attach(SERVO_PIN_R);        // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(pos);                // tell servo to go to position in variable 'pos'
  delay(10);                         // waits 15ms for the servo to reach the position 15


  pos_final_f = EEPROM.read(pos_final_addres_f); // 0 = inner, 1 = outer
  if (pos_final_f <  0         ) {
    pos_final_f = 0; //ギヤポジション0より小を禁止
  }
  if (pos_final_f >  1) {
    pos_final_f = 1; //ギヤポジション1より大を禁止
  }

  for (int k = 0; k <= speed_step ; k++)   //fine tune dataの読み出し
  {
    pos_data_f_E2PR[k] = EEPROM.read(pos_f_memory_addres + k );
    if ( pos_data_f_E2PR[k] >  128) {
      pos_data_f_E2PR[k] =  pos_data_f_E2PR[k] - 256; //offset data EEPROMは0～255なので負数はオフセットさせる。
    }
    pos_data_f[k] =  pos_data_f_ORG[k] + pos_data_f_E2PR[k];
    delay(2);
  }
  delay(2);
  for (int i = 0; i <= 1 ; i++)   //Front offset dataの読み出し
  {
    pos_front_E2PR[i] = EEPROM.read(pos_f_set_memory_addres + i);
    if (pos_front_E2PR[i] >  128) {
      pos_front_E2PR[i] =  pos_front_E2PR[i] - 256; //offset data EEPROMは0～255なので負数はオフセットさせる。
    }
    delay(2);
  }
  pos_f = pos_front[pos_final_f] + pos_front_E2PR[pos_final_f] + pos_data_f[pos_new];
  myservo_f.attach(SERVO_PIN_F);      // attaches the servo on pin 8 to the servo object
  myservo_f.writeMicroseconds(pos_f); // tell servo to go to position in variable 'pos'
  delay(200);                         // waits 15ms for the servo to reach the position 15

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  //  establishContact();  // send a byte to establish contact until receiver responds
  //  MsTimer2::set(1000, servoctl);
  //  MsTimer2::start();
  // Serial.println(pos_new);
  // Setup the button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_f1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_f2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_MODESEL, INPUT_PULLUP);
  pinMode(BUTTON_PIN_MODESEL2, INPUT_PULLUP);

  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_PIN);
  debouncer2.attach(BUTTON_PIN2);
  debouncer.interval(2);
  debouncer2.interval(2);
  debouncer3.attach(BUTTON_PIN_f1);
  debouncer4.attach(BUTTON_PIN_f2);
  debouncer3.interval(2);
  debouncer4.interval(2);
  debouncerX.attach(BUTTON_PIN_MODESEL);
  debouncerX.interval(250);   //0.25secで切り替え
  debouncerY.attach(BUTTON_PIN_MODESEL2);
  debouncerY.interval(250);   //0.25secで切り替え


  //Setup the LED
  pinMode(LED_PIN_MODSEL, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
}




void loop() {
  delay(5);

  if (Serial.available() > 0) {     // get incoming byte:
    serialin();                        //下記のvoid serialinの内容を実行
    Serial.println(pos);              // send a capital ASerial.println('pos');   // send a capital pos
    myservo.writeMicroseconds(pos);                // tell servo to go to position in variable 'pos'
    myservo_f.writeMicroseconds(pos_f);              // tell servo to go to position in variable 'pos_f'
    delay(50);                         // waits 15ms for the servo to reach the position 15
    Serial.println(myservo.read());
    Serial.println(myservo_f.read());
  }

  // Update the debouncer and get the changed state
  boolean changed = debouncer.update();
  boolean changed2 = debouncer2.update();
  boolean changed_f1 = debouncer3.update();
  boolean changed_f2 = debouncer4.update();
  boolean changed_SEL = debouncerX.update();
  boolean changed_SEL2 = debouncerY.update();

  if ( changed_SEL ) {
    // Get the update value
    int value = debouncerX.read();
    if ( value == HIGH) {
      CAL_STAT = false;
      CAL_STAT2 = false;
      pinMode(LED_PIN_MODSEL, OUTPUT);   // fix pinmode // pinMode(LED_PIN_MODSEL, INPUT_PULLUP);
      digitalWrite(LED_PIN_MODSEL, 0 );
      Serial.print(F("CAL STATUS = "));
      Serial.println(CAL_STAT);
      Serial.print(F("CAL STATUS2 = "));
      Serial.println(CAL_STAT2);
      Serial.println(F("CAL MODE END"));
      Serial.println(F(""));
    }
    else  {
      CAL_STAT = true;
      int value = debouncerY.read();
      if ( value == LOW) {
        //CAL_STAT = false;
        CAL_STAT2 = true;
      }
      pinMode(LED_PIN_MODSEL, INPUT_PULLUP);
      //digtalWrite(LED_PIN_MODSEL,HIGH );
      Serial.print(F("CAL STATUS = "));
      Serial.println(CAL_STAT);
      Serial.print(F("CAL STATUS2 = "));
      Serial.println(CAL_STAT2);
      paramter_output();
    }
  }



  if ( changed ) {
    // Get the update value
    int value = debouncer.read();
    if ( value == HIGH) {
      digitalWrite(LED_PIN2, LOW );
      buttonState = 0;
      //Serial.println(F("Button released (state 0)"));
      pos_ctrl_end_TimeStamp = millis();
      Serial.print(F("TimeStamp = "));
      Serial.println(millis());
      Serial.print(F("pos_ctrl_end_TimeStamp = "));
      Serial.println( pos_ctrl_end_TimeStamp);
      //        pos_ctrl_end();
    }
    else  {
      digitalWrite(LED_PIN2, HIGH );
      buttonState = 1;
      //Serial.println(F("Button pressed (state 1)"));
      if (CAL_STAT == true ) {
        if (CAL_STAT2 == true ) {
          if (pos_new == 0 ) {
            pos_data_zero_E2PR = pos_data_zero_E2PR - zero_tune_pitch;
            pos_zero = pos_zero - zero_tune_pitch;
            EEPROM.write(pos_memory_addres_zero, pos_data_zero_E2PR);
            delay(5);
          }
          else {
            pos_shiftstep = pos_shiftstep + 1 ;
            pos_data_step_E2PR = pos_data_step_E2PR + 1;
            EEPROM.write(pos_memory_addres_step, pos_data_step_E2PR);
            delay(5);
          }
          Serial.print(F("pos_zero = "));
          Serial.println(pos_zero);
          Serial.print(F("pos_shiftstep = "));
          Serial.println(pos_shiftstep);
        }
        else {
          pos_data[pos_new] = pos_data[pos_new] + fine_tune_pitch;
        }
      }
      else {
        pos_old = pos_new;
        if ( pos_new < speed_step ) {
          pos_new ++;
          Serial.print(F("pos_new = "));
          Serial.println(pos_new);
        }
        pos_ctrl();
      }
      buttonPressTimeStamp = millis();
    }
  }

  if ( changed2 ) {
    // Get the update value
    int value2 = debouncer2.read();
    if ( value2 == HIGH) {
      digitalWrite(LED_PIN2, LOW );
      buttonState2 = 0;
      //Serial.println(F("Button2 released (state 0)"));
      pos_ctrl_end_TimeStamp = millis();
      Serial.print(F("TimeStamp = "));
      Serial.println(millis());
      Serial.print(F("pos_ctrl_end_TimeStamp = "));
      Serial.println( pos_ctrl_end_TimeStamp);
      //        pos_ctrl_end();
    }
    else {
      digitalWrite(LED_PIN2, HIGH );
      buttonState2 = 1;
      //Serial.println(F("Button2 pressed (state 1)"));
      if (CAL_STAT == true ) {
        if (CAL_STAT2 == true ) {
          if (pos_new == 0 ) {
            pos_data_zero_E2PR = pos_data_zero_E2PR + zero_tune_pitch;
            pos_zero = pos_zero + zero_tune_pitch;
            EEPROM.write(pos_memory_addres_zero, pos_data_zero_E2PR);
            delay(5);
          }
          else {
            pos_shiftstep = pos_shiftstep - 1 ;
            pos_data_step_E2PR = pos_data_step_E2PR - 1;
            EEPROM.write(pos_memory_addres_step, pos_data_step_E2PR);
            delay(5);
          }
          Serial.print(F("pos_zero = "));
          Serial.println(pos_zero);
          Serial.print(F("pos_shiftstep = "));
          Serial.println(pos_shiftstep);
        }
        else {
          pos_data[pos_new] = pos_data[pos_new] - fine_tune_pitch;
        }
      }
      else {
        pos_old = pos_new;
        if ( pos_new > 0 ) {
          pos_new --;
          Serial.print(F("pos_new = "));
          Serial.println(pos_new);
        }
        pos_ctrl();
      }
      buttonPressTimeStamp2 = millis();
    }
  }

  //////* メカ位置最終調整用*//////
  if  ( buttonState == 0 ) {
    if ( millis() - pos_ctrl_end_TimeStamp >= ovsft_delay ) {               // ovsft_delay[msec]
      buttonState = 100;
      Serial.print(F("TimeStamp = "));
      Serial.println(millis());
      pos_ctrl_end();
    }
  }
  if  ( buttonState2 == 0 ) {
    if ( millis() - pos_ctrl_end_TimeStamp >= ovsft_delay ) {               // ovsft_delay[msec]
      buttonState2 = 100;
      Serial.print(F("TimeStamp = "));
      Serial.println(millis());
      pos_ctrl_end();
    }
  }

  /* 多段変速用*/

  if ( CAL_STAT == false ) {                          //　CAL時は多段変速を禁止
    if ( CAL_STAT2 == false ) {                       //　CAL時は多段変速を禁止
      if  ( buttonState == 1 ) {
        if ( millis() - buttonPressTimeStamp >= wait_msec1 ) {
          buttonState = 2;
          //Serial.println(F("Button held for 0.50 seconds (state 2)"));
          pos_old = pos_new;
          if ( pos_new < speed_step ) {
            pos_new ++;
          }
          pos_ctrl();
        }
      }
      if  ( buttonState == 2 ) {
        if ( millis() - buttonPressTimeStamp >= wait_msec2 ) {
          buttonState = 3;
          //Serial.println(F("Button held for 1.0 seconds (state 3)"));
          pos_old = pos_new;
          if ( pos_new < speed_step ) {
            pos_new ++;
          }
          pos_ctrl();
        }
      }
      if  ( buttonState == 3 ) {
        if ( millis() - buttonPressTimeStamp >= wait_msec3 ) {
          buttonState = 100;
          //Serial.println(F("Button held for 1.5 seconds (state 4)"));
          pos_old = pos_new;
          if ( pos_new < speed_step ) {
            pos_new ++;
          }
          pos_ctrl();
        }
      }

      if  ( buttonState2 == 1 ) {
        if ( millis() - buttonPressTimeStamp2 >= wait_msec1 ) {
          buttonState2 = 2;
          //Serial.println(F("Button2 held for 0.50 seconds (state 2)"));
          pos_old = pos_new;
          if ( pos_new  > 0 ) {
            pos_new --;
          }
          pos_ctrl();
        }
      }
      if  ( buttonState2 == 2 ) {
        if ( millis() - buttonPressTimeStamp2 >= wait_msec2 ) {
          buttonState2 = 3;
          //Serial.println(F("Button2 held for 1.0 seconds (state 3)"));
          pos_old = pos_new;
          if ( pos_new  > 0 ) {
            pos_new --;
          }
          pos_ctrl();
        }
      }
      if  ( buttonState2 == 3 ) {
        if ( millis() - buttonPressTimeStamp2 >= wait_msec3 ) {
          buttonState2 = 100;
          //Serial.println(F("Button2 held for 1.5 seconds (state 4)"));
          pos_old = pos_new;
          if ( pos_new  > 0 ) {
            pos_new --;
          }
          pos_ctrl();
        }
      }
    }
  }

  //////* Fメカ変速用*//////
  if ( changed_f1 ) {
    // Get the update value
    int value = debouncer3.read();
    if ( value == HIGH) {
      digitalWrite(LED_PIN2, LOW );
      buttonState3 = 0;
      //Serial.println(F("Button_f1 released (state 0)"));
      //pos_ctrl_end_TimeStamp = millis();
      //      if ( millis() - pos_ctrl_end_TimeStamp >= ovsft_delay ) {               // ovsft_delay[msec]
              pos_ctrl_end();
      //      }
    }
    else  {
      digitalWrite(LED_PIN2, HIGH );
      buttonState3 = 1;
      if (CAL_STAT == true ) {
      }
      if (CAL_STAT2 == true ) {
        pos_front_E2PR[pos_final_f] = pos_front_E2PR[pos_final_f] + 10 ;
        EEPROM.write(pos_f_set_memory_addres + pos_final_f, pos_front_E2PR[pos_final_f]);
        delay(5);
        Serial.print(F("pos_final_f = "));
        Serial.println(pos_final_f);
        Serial.print(F("pos_front_E2PR = "));
        Serial.println(pos_front_E2PR[pos_final_f]);
      }
      else {
        buttonPressTimeStamp3 = millis();
        pos_final_f = 0;
        pos_f = pos_front[pos_final_f] + pos_front_E2PR[pos_final_f] - pos_shiftover_up_f; // + pos_data_f[pos_new]
        Serial.print(F("servo_f position = "));
        Serial.println(pos_f);
        myservo_f.writeMicroseconds(pos_f);
      }
    }
  }

  if ( changed_f2 ) {
    // Get the update value
    int value2 = debouncer4.read();
    if ( value2 == HIGH) {
      digitalWrite(LED_PIN2, LOW );
      buttonState4 = 0;
      //Serial.println(F("Button_f2 released (state 0)"));
      pos_ctrl_end_TimeStamp = millis();
      //      if ( millis() - pos_ctrl_end_TimeStamp >= ovsft_delay ) {               // ovsft_delay[msec]
              pos_ctrl_end();
      //      }
    }
    else {
      digitalWrite(LED_PIN2, HIGH );
      buttonState4 = 1;
      //Serial.println(F("Button_f2 pressed (state 1)"));
      if (CAL_STAT == true ) {
      }
      if (CAL_STAT2 == true ) {
        pos_front_E2PR[pos_final_f] = pos_front_E2PR[pos_final_f] - 10 ;
        EEPROM.write(pos_f_set_memory_addres + pos_final_f, pos_front_E2PR[pos_final_f]);
        delay(5);
        Serial.print(F("pos_final_f = "));
        Serial.println(pos_final_f);
        Serial.print(F("pos_front_E2PR = "));
        Serial.println(pos_front_E2PR[pos_final_f]);
      }
      else {
        buttonPressTimeStamp4 = millis();
        pos_final_f = 1;
        pos_f = pos_front[pos_final_f] - pos_shiftover_down_f; // + pos_data_f[pos_new]
        Serial.print(F("servo_f position = "));
        Serial.println(pos_f);
        myservo_f.writeMicroseconds(pos_f);
      }
    }
  }


} // LOOP 完了



//////////////SUB CKT/////////////////////////

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println('A');   // send a capital A
    delay(300);
  }
}

void serialin() {
  Serial.print(F("serial in" ));
  for (int i = 0; i <= 2; i++) //iが0～2まで変動するので、合計3桁分
  {
    //          Serial.println(i);
    input[i] = Serial.read(); //一桁づつ入れてゆく
    //          delay(100);
  }
  Serial.println(input);
  int buf = atoi(input); //シリアル入力された文字列をint型に変換
  //       if(buf<=180&&buf>=0)//安全のため、PWMで扱える0～255の範囲の時のみPWM出力の値に反映
  //       {
  pos = buf;
  Serial.print(F("serial input = "));
  Serial.println(pos);
  //       }

  //      Serial.flush();
}

///////////////POSITION CTRL///////////////////
void pos_ctrl()
{
  pos_direction = pos_new - pos_old;
  if ( pos_direction > 0 ) {                                        // up stroke
    pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new] - pos_shiftover_up;   // shift over degree
    Serial.print(F("servo position = "));
    Serial.println(pos);
    //delay(130);
    myservo.writeMicroseconds(pos);
  }
  if ( pos_direction < 0 ) {                                          // down stroke
    pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new] + pos_shiftover_down;  // shift over degree
    Serial.print(F("servo position = "));
    Serial.println(pos);
    // delay(130);
    myservo.writeMicroseconds(pos);
  }
  if ( pos_new == speed_step ) {                                        //
    pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new] - pos_shiftover_up;   // shift over degree
    Serial.print(F("servo position = "));
    Serial.println(pos);
    myservo.writeMicroseconds(pos);
  }
  if ( pos_new == 0          ) {                                        //
    pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new] + pos_shiftover_down;    // shift over degree
    Serial.print(F("servo position = "));
    Serial.println(pos);
    myservo.writeMicroseconds(pos);
  }


}

void pos_ctrl_end() {
  pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new] - pos_shiftover_up_tune;               // set position
  myservo.writeMicroseconds(pos);                                             // tell servo to go to position in variable 'pos'
  delay(ovsft_delay2);                                                         // ovsft_delay[msec]
  pos = pos_zero - pos_shiftstep * pos_new - pos_data[pos_new];               // set position
  myservo.writeMicroseconds(pos);                                             // tell servo to go to position in variable 'pos'
  pos_f = pos_front[pos_final_f] + pos_front_E2PR[pos_final_f] + pos_data_f[pos_new];    // set position フロントの段数＋リヤの段数によるトリム
  myservo_f.writeMicroseconds(pos_f);                                         // tell servo to go to position in variable 'pos_f'

  pos_final = pos_new;
  EEPROM.write(pos_final_addres, pos_final);
  EEPROM.write(pos_final_addres_f, pos_final_f);    //pos_final_f : 0 = INNER 1 = OUTER
  delay(5);

  pos_data_E2PR[pos_new] = pos_data[pos_new] - pos_data_ORG[pos_new];
  EEPROM.write(pos_memory_addres + pos_new, pos_data_E2PR[pos_new]);
  delay(5);
  pos_data_f_E2PR[pos_new] = pos_data_f[pos_new] - pos_data_f_ORG[pos_new];
  EEPROM.write(pos_f_memory_addres + pos_new, pos_data_f_E2PR[pos_new]);
  delay(5);

  Serial.println();
  Serial.println(F("//////// REAR DATA/////////////////////////////////////"));
  Serial.print(F("servo rear position = "));
  Serial.println(pos);
  pos_final = EEPROM.read(pos_final_addres);
  Serial.print(F("EEPROM NEW REAR POS = "));
  Serial.println(pos_final);
  //Serial.print(F("servo REAR fine tune offset EEPROM = "));
  //Serial.println(pos_data_E2PR[pos_new]);
  int offset_data_EEPROM = EEPROM.read(pos_memory_addres + pos_final);
  Serial.print(F("EEPROM new fine tune offset = "));
  Serial.println(offset_data_EEPROM);
  Serial.println();

  Serial.println(F("//////// FRONT DATA/////////////////////////////////////"));
  Serial.print(F("servo front position = "));
  Serial.println(pos_f);
  pos_final_f = EEPROM.read(pos_final_addres_f);
  Serial.print(F("EEPROM NEW FRONT POS = "));
  Serial.println(pos_final_f);
  //Serial.print(F("servo FRONT fine tune offset EEPROM = "));
  //Serial.println(pos_data_f_E2PR[pos_new]);
  int offset_data_f_EEPROM = EEPROM.read(pos_f_memory_addres + pos_final);
  Serial.print(F("EEPROM nww fine tune offset = "));
  Serial.println(offset_data_f_EEPROM);
  Serial.println();
}

void paramter_output() {
  //  Serial.println(F(""));
  Serial.println(F("////////  MCU PIN ASSIGN /////////////////////////////////////"));
  Serial.print(F("FRONT SERVO PIN = "));
  Serial.println(SERVO_PIN_F);
  Serial.print(F("REAR SERVO PIN = "));
  Serial.println(SERVO_PIN_R);
  Serial.print(F("REAR LOW DOUN BUTTON_PIN = "));
  Serial.println(BUTTON_PIN);
  Serial.print(F("REAR High UP BUTTON_PIN2 = "));
  Serial.println(BUTTON_PIN2);
  Serial.print(F("FRONT LOW DOUN BUTTON_PIN_f1 = "));
  Serial.println(BUTTON_PIN_f1);
  Serial.print(F("FRONT High UP BUTTON_PIN_f2 = "));
  Serial.println(BUTTON_PIN_f2);
  Serial.print(F("SETTING MODE SELECT = "));
  Serial.println(BUTTON_PIN_MODESEL);
  Serial.print(F("SETTING MODE2 SELECT = "));
  Serial.println(BUTTON_PIN_MODESEL2);

  Serial.print(F("SHIFT INDICATOR LED = "));
  Serial.println(LED_PIN2);
  Serial.print(F("SETTING MODE INDICATOR LED = "));
  Serial.println(LED_PIN_MODSEL);

  Serial.println(F(""));
  Serial.println(F("//////// REAR PARAMETERS /////////////////////////////////////"));
  Serial.print(F("speed_step = "));
  Serial.print(speed_step);
  Serial.println(F("             //8=9SPEED 9=10SPEED 10=11SPEED"));
  Serial.print(F("pos_zero_org = "));
  Serial.println(pos_zero_org);
  Serial.print(F("pos_zero = "));
  Serial.print(pos_zero);
  Serial.println(F("             // shift 0 position : about 2000~1800"));
  Serial.print(F("pos_shiftstep_org = "));
  Serial.println(pos_shiftstep_org);
  Serial.print(F("pos_shiftstep = "));
  Serial.print(pos_shiftstep);
  Serial.println(F("             //shift step"));
  Serial.print(F("pos_shiftover_up = "));
  Serial.print(pos_shiftover_up);
  Serial.println(F("             // shift over step"));
  Serial.print(F("pos_shiftover_down = "));
  Serial.print(pos_shiftover_down);
  Serial.println(F("             // shift over step"));
  Serial.print(F("wait_msec1 = "));
  Serial.print(wait_msec1);
  Serial.println(F("[msec]"));
  Serial.print(F("wait_msec2 = "));
  Serial.print(wait_msec2);
  Serial.println(F("[msec]"));
  Serial.print(F("wait_msec3 = "));
  Serial.print(wait_msec3);
  Serial.println(F("[msec]"));
  Serial.print(F("fine_tune_pitch = "));
  Serial.print(fine_tune_pitch);
  Serial.println(F("             //fine tune pitch for setting mode"));

  Serial.print(F("pos_data_ORG[11] = "));
  for (int x = 0; x <= 10; x++) {
    Serial.print(pos_data_ORG[x]);
    Serial.print(F(","));
  }
  Serial.println(F(""));

  Serial.print(F("pos_data_E2PR[11] = "));
  for (int x = 0; x <= 10; x++) {
    Serial.print(pos_data_E2PR[x]);
    Serial.print(F(","));
  }
  Serial.println(F(""));

  Serial.println(F(""));
  Serial.println(F("//////// FRONT PARAMETERS /////////////////////////////////////"));
  Serial.print(F("pos_front[2] = "));
  for (int x = 0; x <= 1; x++) {
    Serial.print(pos_front[x]);
    Serial.print(F(","));
  }
  Serial.println(F("             // {inner, outer}"));
  Serial.print(F("pos_front_E2PR[2] = "));
  for (int x = 0; x <= 1; x++) {
    Serial.print(pos_front_E2PR[x]);
    Serial.print(F(","));
  }
  Serial.println(F("             //E2PR {inner, outer}"));
  Serial.print(F("pos_shiftover_up_f = "));
  Serial.print( pos_shiftover_up_f);
  Serial.println(F("             // shift over step "));
  Serial.print(F("pos_shiftover_down_f = "));
  Serial.print(pos_shiftover_down_f);
  Serial.println(F("             // shift over step "));

  Serial.print(F("pos_data_f_ORG[11] = "));
  for (int x = 0; x <= 10; x++) {
    Serial.print(pos_data_f_ORG[x]);
    Serial.print(F(","));
  }
  Serial.println(F(""));

  Serial.print(F("pos_data_f_E2PR[11] = "));
  for (int x = 0; x <= 10; x++) {
    Serial.print(pos_data_f_E2PR[x]);
    Serial.print(F(","));
  }
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("//////// You can set this position. ////////////"));
  Serial.print(F("SETTING POSITION REAR = "));
  Serial.println(pos_final);
  Serial.print(F("SETTING POSITION FRONT = "));
  Serial.println(pos_final_f);
  Serial.println(F(""));
}
