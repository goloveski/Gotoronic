// 160211各ギヤごとにデータを持たせる。-> pos_data[10]
// 160222 save gear position to EEPROM
// 160308 delete// 160222 added single LINE 2 signal SW system
// 160222 added EEPROM status limit
// 160308 added EEPROM offset data

// SERVO REAR
// Red = +6v
// Brown = GND
// Orange = Signal (pin 9 for this code)

// SERVO FRONT
// Red = +6v
// Brown = GND
// Orange = Signal (pin 8 for this code)

/* delete
      // single LINE 2 signal SW system
      // shift UP   SIGNAL(PIN4)SW--35Kohm--GND about 1/2 VDD
      // shift DOWN SIGNAL(PIN4)SW-- 0ohm --GND 
      // PIN4 
      // tune adc_val, ana_read_dly, for sensitivity
*/

#include <EEPROM.h>
#include <Servo.h> 
#include <Bounce2.h>
#include <MsTimer2.h> 

//////////////////////////////// MCU PIN ASSIGN START ///////////////////////////////////////////
#define BUTTON_PIN 4 //REAR
#define BUTTON_PIN2 5 //REAR
#define BUTTON_PIN_f1 6  //FRONT
#define BUTTON_PIN_f2 7  //FRONT
#define SERVO_PIN_F 8  // SERVO F
#define SERVO_PIN_R 9  // SERVO R

#define LED_PIN2 10
#define LED_PIN 13          // LED goes off when signal is received
#define BUTTON_PIN_MODESEL 10  //MODE SELECT
////////////////////////////////  MCU PIN ASSIGN END ///////////////////////////////////////////



// Instantiate a Bounce object
Bounce debouncer  = Bounce(); 
Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); 
Bounce debouncer4 = Bounce();
Bounce debouncerX = Bounce();

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



//ロータリーエンコーダ用の設定
volatile int state = 0;  //ロータリーエンコーダで操作する値
volatile boolean Flag_A=true;  //ロータリーエンコーダA端子の状態変化を許すフラグ
volatile boolean Flag_B=false; //ロータリーエンコーダB端子の状態変化を許すフラグ
int RotEncState = LOW; //ロータリーエンコーダで入力したことを示すフラグ

Servo myservo;  // create servo object to control a servo 
Servo myservo_f; // a maximum of eight servo objects can be created 
char input[2];  //シリアル通信の文字列格納用
char str[0];    //シリアル通信の文字列格納用
int pos = 0;    // variable to store the servo position
int pos_f = 0;  // variable to store the servo position 
          //int analogvalue1; //ボタン１のアナログ値
          //int analogvalue2; //ボタン１のアナログ値
          //int adc_val[2] ={400, 700}; //AD閾値
          //char adc_val_position[3][5] = {"LOW", "MID","HIGH"};//AD閾値
          //int NUM_ADC_DIV = 2;//AD閾値
          //int SWstatus = 0;//ADステータス
          //int SWstatus_new = 0;//ADステータス
          //int SWstatus_old = 0;//ADステータス
          //int SW_short_status_new = 0;//ADステータス
          //int SW_short_status_old = 0;//ADステータス
          //int ana_read_dly = 10; //delay anarog read[ms]

//////////////////////////////// EEPROM PARAMETER START ///////////////////////////////////////////
int pos_final_addres = 10; //final gear position save address
int pos_final; //final gear position
int pos_final_addres_f = 1; //final gear position save address front
int pos_final_f; //final gear position front
int pos_zero_memory_addres = 30; //zero position memory addres 30 
int pos_memory_addres = 11; //offsetmemory start addres 11 12 13 14 15 16 17 18 19 20 21
//int pos_memory[10]; //zero position memory


//////////////////////////////// REAR SHIFT SETTING PARAMETER START ///////////////////////////////////////////

int speed_step = 8;            //8=9速 9=10速 10=11速
int pos_zero0 = 1925;          // shift 0 position 180deg ->1st gear direction, 0deg ->10th gear direction, 
int pos_zero = 1925;           // shift 0 position 180deg ->1st gear direction, 0deg ->10th gear direction, 
int pos_shiftstep = 68;        // shift step degree 56->68 @160404 for 9speed
int pos_data[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 11speed fine tuninng data

int pos_shiftover_up = 40;   // shift over degree
int pos_shiftover_down = 40; // shift over degree
int ovsft_delay = 500;       // over shift delay [msec]
int sft_delay = 20;          // shift delay per gear pos [msec]

int wait_msec1 =  500;     //多段変則設定時間1[msec]
int wait_msec2 = 1000;     //多段変則設定時間2[msec]
int wait_msec3 = 1500;     //多段変則設定時間3[msec]

//////////////////////////////// REAR SHIFT SETTING PARAMETER END ///////////////////////////////////////////

int pos_old;           // gear position
int pos_new;           // gear position
int pos_direction  = 1;     // shift direction
int ovsft_delay_tot = 100;    // over shift delay msec


//////////////////////////////// FRONT SHIFT SETTING PARAMETER START  ///////////////////////////////////////////

int pos_zero_f = 80;          // shift 0 position 180deg ->1st gear direction, 0deg ->10th gear direction, 
int pos_inner = 70;      //80->70 @150809
int pos_outer = 150;     //120->120 @150809
int pos_shiftstep_f = 5;      // shift step degree
int pos_shiftover_up_f = 40;   // shift over degree 15->20 @150809
int pos_shiftover_down_f = 40; // shift over degree 15->40 @150809
int ovsft_delay_f = 2000;      // over shift delay [msec]
int sft_delay_f = 20;         // shift delay per gear pos [msec]

//////////////////////////////// FONT SHIFT SETTING PARAMETER END ///////////////////////////////////////////

int pos_old_f  = 0;           // gear position
int pos_new_f  = 0;           // gear position
int pos_direction_f  = 1;     // shift direction
int ovsft_delay_tot_f = 100;    // over shift delay msec

 
void setup() 
{ 

  //pos_zero = pos_zero0;
  pos_zero =  pos_zero + EEPROM.read(pos_zero_memory_addres);  // pos zero memory addres
    
  pos_final = EEPROM.read(pos_final_addres);
  if (pos_final <  0         ){ pos_final = 0;} //ギヤポジション0より小を禁止
  if (pos_final >  speed_step){ pos_final = 0;} //ギヤポジション9より大を禁止
  pos_new = pos_final;
  pos = pos_zero - pos_shiftstep * pos_new + pos_data[pos_new];
  
  for(int n=2;n<= speed_step+1 ;n++)
      {
        pos_data[n] = EEPROM.read(pos_memory_addres + n -1 );
        if (pos_data[n] <  0){ pos_data[n] = 0;} //offset data 0より小を禁止
        if (pos_data[n] >  200){ pos_data[n] = 0;} //offset data 200より大を禁止
        EEPROM.write(pos_memory_addres + n -1,pos_data[n]);
        delay(2);
      }
  delay(2);
  myservo.attach(SERVO_PIN_R);        // attaches the servo on pin 9 to the servo object 
  myservo_f.attach(SERVO_PIN_F);      // attaches the servo on pin 8 to the servo object 
  myservo.writeMicroseconds(pos);                // tell servo to go to position in variable 'pos'
  delay(10);                         // waits 15ms for the servo to reach the position 15
  pos_f = pos_zero_f;         //set zero position
  myservo_f.write(pos_f);                // tell servo to go to position in variable 'pos'
  delay(200);                         // waits 15ms for the servo to reach the position 15
  Serial.begin(9600);
     // while (!Serial) {
     //   ; // wait for serial port to connect. Needed for Leonardo only
     // }
  //  establishContact();  // send a byte to establish contact until receiver responds 
   //  MsTimer2::set(1000, servoctl);
   //  MsTimer2::start();
  Serial.println(pos_new); 
  // Setup the button
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(BUTTON_PIN2,INPUT_PULLUP);
  pinMode(BUTTON_PIN_f1,INPUT_PULLUP);
  pinMode(BUTTON_PIN_f2,INPUT_PULLUP);
  pinMode(BUTTON_PIN_MODESEL,INPUT_PULLUP);
  
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
  debouncerX.interval(500);   //0.5secで切り替え
  
  //Setup the LED
  pinMode(LED_PIN2,OUTPUT);

//  pinMode(2, INPUT_PULLUP);  //デジタルピン2をプルアップ入力にする
//  pinMode(3, INPUT_PULLUP);  //デジタルピン3をプルアップ入力にする
//  attachInterrupt(0, Fall_A  , FALLING);//デジタルピン2の電圧が下がった時の処理関数を登録
//  attachInterrupt(1, Change_B, CHANGE );//デジタルピン3の電圧が変化した時の処理関数を登録
   
} 
 
void loop() {  
  delay(5);

  if (Serial.available() > 0) {     // get incoming byte:
  serialin();                        //下記のvoid serialinの内容を実行
  Serial.println(pos);              // send a capital ASerial.println('pos');   // send a capital pos
  myservo.writeMicroseconds(pos);                // tell servo to go to position in variable 'pos'
  myservo_f.write(pos_f);              // tell servo to go to position in variable 'pos'
  delay(50);                         // waits 15ms for the servo to reach the position 15
  Serial.println(myservo.read());
  Serial.println(myservo_f.read());
  }

//   Serial.print("START POSITION = ");
//   Serial.println(pos_final);

 // Update the debouncer and get the changed state
  boolean changed = debouncer.update();
  boolean changed2 = debouncer2.update();
  boolean changed_f1 = debouncer3.update();
  boolean changed_f2 = debouncer4.update();
  boolean changed_SEL = debouncerX.update();

 if ( changed ) {
       // Get the update value
    int value = debouncer.read();
    if ( value == HIGH) {
       digitalWrite(LED_PIN2, HIGH );
       buttonState = 0;
       Serial.println("Button released (state 0)");
       pos_ctrl_end();
   }
   else  {
         digitalWrite(LED_PIN2, LOW );
         buttonState = 1;
         Serial.println("Button pressed (state 1)");
         buttonPressTimeStamp = millis();
         pos_old = pos_new;
         if( pos_new < speed_step ) {pos_new ++;}
         pos_ctrl();
     }
  }
 
  if ( changed2 ) {
       // Get the update value
    int value2 = debouncer2.read();
    if ( value2 == HIGH) {
       digitalWrite(LED_PIN2, HIGH );
       buttonState2 = 0;
       Serial.println("Button2 released (state 0)");
       pos_ctrl_end();
   }
   else {
         digitalWrite(LED_PIN2, LOW );
         buttonState2 = 1;
         Serial.println("Button2 pressed (state 1)");
         buttonPressTimeStamp2 = millis();
         pos_old = pos_new;
         if( pos_new >0 ) {pos_new --;}
         pos_ctrl();
        }
  }
 
        
        // //Read Analog SW   
        //   int n;
        //   for (n = 0; n < 10; n++){
        //      if (SW_short_status_old != SW_short_status_new){
        //          SW_short_status_old = SW_short_status_new;
        //          delay(10); 
        //          break;
        //          }
        //          
        //       ReadAna1() ;
        //       int k;
        //       for (k = 0; k < NUM_ADC_DIV; k++){
        //            if (analogvalue1 < adc_val[k]){
        //               break;
        //               }   
        //       }
        //       SW_short_status_new = k;
        //       k = 0;
        //     }
        //   SWstatus_new = SW_short_status_new; 
        ////   Serial.print("ANALOG SW POSITION = ");
        ////   Serial.println( adc_val_position[SWstatus_new]);
        //   delay(10); 
        //
        //if (SWstatus_old != SWstatus_new){
        //      SWstatus_old = SWstatus_new;
        //
        //        switch(SWstatus_new){
        //            case 0:   
        //               digitalWrite(LED_PIN2, LOW );
        //               buttonState2 = 1;
        //               buttonState = 0;
        //               Serial.println("Button pressed (state LOW)");
        //               buttonPressTimeStamp2 = millis();
        //               pos_old = pos_new;
        //               if( pos_new > 0 ) {pos_new --;}
        //               pos_ctrl();
        //               break;
        //
        //            case 1:
        //               digitalWrite(LED_PIN2, LOW );
        //               buttonState = 1;
        //               buttonState2 = 0;
        //               Serial.println("Button pressed (state MID)");
        //               buttonPressTimeStamp = millis();
        //               pos_old = pos_new;
        //               if( pos_new < 9 ) {pos_new ++;}
        //               pos_ctrl();
        //               break;  
        //    
        //            case 2:
        //               // Get the update value
        //               digitalWrite(LED_PIN2, HIGH );
        //               buttonState = 0;
        //               buttonState2 = 0;
        //               Serial.println("Button released (state HIGH)");
        //               pos_ctrl_end();
        //               break;    
        //        } 
        //   }
   
 /* 多段変速用*/
 
  if  ( buttonState == 1 ) {
    if ( millis() - buttonPressTimeStamp >= wait_msec1 ) {
        buttonState = 2;
        Serial.println("Button held for 0.50 seconds (state 2)");
        pos_old = pos_new;
        if( pos_new < speed_step ) {pos_new ++;}
        pos_ctrl();
       }
  }
  if  ( buttonState == 2 ) {
    if ( millis() - buttonPressTimeStamp >= wait_msec2 ) {
        buttonState = 3;
        Serial.println("Button held for 1.0 seconds (state 3)");
        pos_old = pos_new;
        if( pos_new < speed_step ) { pos_new ++;}
        pos_ctrl();
        }
  } 
   if  ( buttonState == 3 ) {
    if ( millis() - buttonPressTimeStamp >= wait_msec3 ) {
        buttonState = 0;
       Serial.println("Button held for 1.5 seconds (state 4)");
       pos_old = pos_new;
       if( pos_new < speed_step ) {pos_new ++;}
           pos_ctrl();
    }
  } 
 
  if  ( buttonState2 == 1 ) {
    if ( millis() - buttonPressTimeStamp2 >= wait_msec1 ) {
        buttonState2 = 2;
       Serial.println("Button2 held for 0.50 seconds (state 2)");
       pos_old = pos_new;
       if( pos_new  >0 ) {pos_new --;}
           pos_ctrl();
       }
  }
  if  ( buttonState2 == 2 ) {
    if ( millis() - buttonPressTimeStamp2 >= wait_msec2 ) {
        buttonState2 = 3;
       Serial.println("Button2 held for 1.0 seconds (state 3)");
       pos_old = pos_new;
       if( pos_new  >0 ) {pos_new --;}
           pos_ctrl();
       }
  } 
  if  ( buttonState2 == 3 ) {
    if ( millis() - buttonPressTimeStamp2 >= wait_msec3 ) {
        buttonState2 = 0;
       Serial.println("Button2 held for 1.5 seconds (state 4)");
       pos_old = pos_new;
       if( pos_new  >0 ) { pos_new --;}
           pos_ctrl();
       }
  } 


 /* Fメカ変速用*/  
if ( changed_f1 ) {
     // Get the update value
  int value = debouncer3.read();
  if ( value == HIGH) {
     digitalWrite(LED_PIN2, HIGH );
     buttonState3 = 0;
     Serial.println("Button_f1 released (state 0)");
     pos_f = pos_inner + pos_shiftover_down_f;   // shift over degree 
     Serial.print("servo_f position = ");
     Serial.println(pos_f);
     myservo_f.write(pos_f); 
     
 }
 else  {
       digitalWrite(LED_PIN2, LOW );
       buttonState3 = 1;
       Serial.println("Button_f1 pressed (state 1)");
       buttonPressTimeStamp3 = millis();
       pos_old_f = pos_new_f;
       pos_new_f = pos_inner;
       pos_f = pos_inner;   // shift over degree 
       Serial.print("servo_f position = ");
       Serial.println(pos_f);
       myservo_f.write(pos_f); 
   }
}

if ( changed_f2 ) {
     // Get the update value
  int value2 = debouncer4.read();
  if ( value2 == HIGH) {
     digitalWrite(LED_PIN2, HIGH );
     buttonState4 = 0;
     Serial.println("Button_f2 released (state 0)");
     pos_f = pos_outer - pos_shiftover_up_f;  // shift over degree
     Serial.print("servo_f position = ");
     Serial.println(pos_f);
     myservo_f.write(pos_f);   
 }
 else {
       digitalWrite(LED_PIN2, LOW );
       buttonState4 = 1;
       Serial.println("Button_f2 pressed (state 1)");
       buttonPressTimeStamp4 = millis();
       pos_old_f = pos_new_f;
       pos_new_f = pos_outer;
       pos_f = pos_outer;  // shift over degree
       Serial.print("servo_f position = ");
       Serial.println(pos_f);
       myservo_f.write(pos_f);
      }
}


// if ( changed_SEL ) {                //setting mode
//       // Get the update value
//    int value0 = debouncerX.read();
//    if ( value0 == HIGH) { 
//      digitalWrite(LED_PIN2, LOW );
//      Serial.println("SETTING MODE END");
//      }
//   else  {
//         digitalWrite(LED_PIN2, HIGH );
//         Serial.println("SETTING MODE START");
//         if ( changed ) {
//                   int value1 = debouncer.read();
//                   if ( value1 == HIGH) {}
//                   else  {
//                         Serial.println("tune +1");
//                         pos_zero = pos_zero + 100;
//                         if (pos_new == 0) {
//                            EEPROM.write(pos_zero_memory_addres, pos_zero - pos_zero0);                //zero のときはpos_zeroを更新(pos_zero0との差分を格納）
//                            pos_data[pos_new] = 0;
//                         }
//                         else {
//                            EEPROM.write(pos_memory_addres + pos_new , pos_data[pos_new]);
//                         }
//                         pos_ctrl_end();
//                   }
//        }
//        if ( changed2 ) {
//                   int value2 = debouncer2.read();
//                   if ( value2 == HIGH) {}
//                   else  {
//                         Serial.println("tune -1");
//                         
//                          if (pos_new == 0) {
//                            pos_zero = pos_zero + 100;
//                            EEPROM.write(pos_zero_memory_addres, pos_zero - pos_zero0);                //zero のときはpos_zeroを更新(pos_zero0との差分を格納）
//                            pos_data[pos_new] = 0;
//                         }
//                         else {
//                            pos_data[pos_new] = pos_data[pos_new]+100;
//                            EEPROM.write(pos_memory_addres + pos_new , pos_data[pos_new]);
//                         }
//                         pos_ctrl_end();
//                   }
//        }  
//   }      
//  }



} // LOOP 完了



//////////////SUB CKT/////////////////////////

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println('A');   // send a capital A
    delay(300);
  }
}

void serialin() {
    Serial.print("serial in" );
    for(int i=0;i<=2;i++)//iが0～2まで変動するので、合計3桁分
        {
//          Serial.println(i);
          input[i]=Serial.read(); //一桁づつ入れてゆく
//          delay(100); 
        }
       Serial.println(input); 
       int buf=atoi(input);//シリアル入力された文字列をint型に変換
//       if(buf<=180&&buf>=0)//安全のため、PWMで扱える0～255の範囲の時のみPWM出力の値に反映
//       {
         pos=buf;
        Serial.print("serial input = ");
        Serial.println(pos);
//       }
 
//      Serial.flush();  
 }

        /////////////////READ ANALOG///////////////////
        //void ReadAna1() {
        //    analogvalue1 = analogRead(A6);
        ////      Serial.print("analogvalue1 = ");
        ////      Serial.println(analogvalue1);
        //    delay(ana_read_dly);  //wait ana_read_dly msec
        //    }

///////////////POSITION CTRL///////////////////
void pos_ctrl() 
{
        pos_direction = pos_new - pos_old;
        ovsft_delay_tot = abs(pos_direction) * sft_delay + ovsft_delay;
//        Serial.println(ovsft_delay_tot);
        if( pos_direction > 0 ) {                                         // up stroke
           pos = pos_zero - pos_shiftstep * pos_new + pos_data[pos_new] - pos_shiftover_up;   // shift over degree
           Serial.print("servo position = ");
           Serial.println(pos);
//delay(130);
           myservo.writeMicroseconds(pos); 
//           delay(ovsft_delay_tot);                                         // waits Xmsec for the servo to reach the position 
//           pos = pos_zero - pos_shiftstep * pos_new;                       // shift over degree
//           myservo.writeMicroseconds(pos);                                             // tell servo to go to position in variable 'pos'
        }
       if( pos_direction < 0 ) {                                           // down stroke 
           pos = pos_zero - pos_shiftstep * pos_new + pos_data[pos_new] + pos_shiftover_down;  // shift over degree
           Serial.print("servo position = ");
           Serial.println(pos);
// delay(130);
           myservo.writeMicroseconds(pos); 
//           delay(ovsft_delay_tot);                                         // waits Xmsec for the servo to reach the position 
//           pos = pos_zero - pos_shiftstep * pos_new;                       // shift over degree
//           myservo.writeMicroseconds(pos);                                             // tell servo to go to position in variable 'pos'
        }
//       Serial.println(pos);
//       Serial.println(myservo.read());
//       delay(15);                                                          // waits 15ms for the servo to reach the position 
   if( pos_new == speed_step ) {                                         //
           pos = pos_zero - pos_shiftstep * pos_new + pos_data[pos_new] - pos_shiftover_up;   // shift over degree
           myservo.writeMicroseconds(pos); 
   }
   if( pos_new == 0          ) {                                         // 
           pos = pos_zero - pos_shiftstep * pos_new + pos_data[pos_new] + pos_shiftover_down;    // shift over degree
           myservo.writeMicroseconds(pos); 
   }


}

void pos_ctrl_end() {
           pos = pos_zero - pos_shiftstep * pos_new + pos_data[pos_new];                       // shift over degree        
           myservo.writeMicroseconds(pos);                                             // tell servo to go to position in variable 'pos' 
           pos_final = EEPROM.read(pos_final_addres);
 //          delay(1000);
           //Serial.print("EEPROM OLD = ");
           //Serial.println(pos_final);
           pos_final = pos_new;
           EEPROM.write(pos_final_addres, pos_final);
           delay(5);
           Serial.print("servo position = ");
           Serial.println(pos);
           Serial.print("servo offset = ");
           Serial.println(pos_data[pos_new]);
           //Serial.println(myservo.read());
           Serial.print("gear position = ");
           Serial.println(pos_new);
           pos_final = EEPROM.read(pos_final_addres);
           Serial.print("EEPROM NEW = ");
           Serial.println(pos_final);
           
}


///////////////POSITION CTRL FRONT///////////////////
void servo_move()
{
  myservo_f.write(pos_f);
      MsTimer2::stop();
      Serial.println("TimerDelay");
      Serial.println(myservo_f.read());
}


void pos_ctrl_f() 
{
        pos_direction_f = pos_new_f - pos_old_f;
        ovsft_delay_tot_f = abs(pos_direction_f) * sft_delay_f + ovsft_delay_f;
        Serial.println(ovsft_delay_tot_f);
        if( pos_direction_f > 0 ) {                                         // up stroke
           pos_f = pos_inner - pos_shiftover_up_f;   // shift over degree 
           Serial.println(pos_f);
// delay(130);
           myservo_f.write(pos_f); 
           //delay(ovsft_delay_tot_f);                                         // waits Xmsec for the servo to reach the position 
           if( RotEncState = HIGH){
           RotEncState = LOW;  
           pos_f = pos_inner;                       // shift over degree
           MsTimer2::set(ovsft_delay_tot_f,servo_move); // 500msごとにオンオフ
           MsTimer2::start();
           // myservo_f.write(pos_f);                                             // tell servo to go to position in variable 'pos'
           }
        }
       if( pos_direction_f < 0 ) {                                           // down stroke 
           pos_f = pos_outer + pos_shiftover_down_f;  // shift over degree
           Serial.println(pos_f);
// delay(130);
           myservo_f.write(pos_f); 
           //delay(ovsft_delay_tot_f);            // waits Xmsec for the servo to reach the position 
          if( RotEncState = HIGH){
           RotEncState = LOW;  
           pos_f = pos_outer;                       // shift over degree
           MsTimer2::set(ovsft_delay_tot_f,servo_move); // 500msごとにオンオフ
           MsTimer2::start();
           //myservo_f.write(pos_f);                                             // tell servo to go to position in variable 'pos'
          }
        }
       Serial.println(pos_f);

       delay(1);                                                          // waits 15ms for the servo to reach the position 
}

//void pos_ctrl_end_f() {
//       //pos_f = pos_zero_f - pos_shiftstep_f * pos_new_f;
//       pos_f = pos_zero_f;
//       myservo_f.writeMicroseconds(pos_f);                                             // tell servo to go to position in variable 'pos' myservoy->myservo_f 2015/2/26     
//       Serial.println(myservo_f.read());
//       Serial.print("Front gear position = ");
//       Serial.println(pos_new_f); 
//}

/////////////////ROT_ENC CTRL FRONT///////////////////
//void Fall_A() {  //デジタルピン2の電圧が下がった時
//
//if(!Flag_A){return;}  //A端子の認識が許されない時は戻る
//Flag_B=true;  //B端子の状態変化の監視を有効にする
//Flag_A=false; //A端子の状態変化の監視を無効にする
//
//}
//void Change_B() {  //デジタルピン3の電圧が変化した時
//  if(!Flag_B){return;} //B端子の認識が許されない時は戻る
//  if(HIGH == digitalRead(2)){ //既にA端子がHIGHならば
//        if(HIGH == digitalRead(3)){
//          state--;
//                  Serial.println(state);  //シリアルで値を送信
//          pos_old_f = pos_new_f;
//          pos_new_f = pos_inner;
//          pos_ctrl_f();
//          }else{
//          state++;
//            Serial.println(state);  //シリアルで値を送信
//          pos_old_f = pos_new_f;
//          pos_new_f = pos_outer;
//          RotEncState = HIGH;     //ロータリーエンコーダで入力したことを示すフラグ
//          pos_ctrl_f();
//          }  //現在の電圧から加減算を行う
//        Flag_B=false; //B端子の状態変化の監視を無効にする
//        Flag_A=true;  //A端子の状態変化の監視を有効にする
//
//        }
//
//}
