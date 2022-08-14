#include "Arduino.h"
#include <avr/sleep.h>
#include "MPU9250.h" 
#include "U8glib.h"

U8GLIB_SSD1306_128X64 display(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST); // Fast I2C / TWI 

MPU9250 IMU(Wire,0x68);
bool check_9dof = false;
int status;
float gyrox, gyroy, gyroz, bufferx, buffery, bufferz, errorx, errory, errorz;
float deltat = 0.0f;
uint32_t lastUpdate = 0;
uint32_t Now = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

float angle, angleTemp, angle_x, angle_y, angle_z;

#define INTERRUPT_PWR_BUTTON_PIN 2
#define INTERRUPT_MODE_BUTTON_PIN 3
int ButtonCounter = 0;
bool isSleeping = false;
int intCounter = 0;

int chargePlugPin = 12;
int BatteryReadPin = A0;
float DividedVoltage;
float BatteryVoltage;
int BatteryCounter;
int chargePercent = 100;


// This is the area inside the battery bitmap, in pixels
#define CHARGE_AREA_START_X     108
#define CHARGE_AREA_START_Y     2
#define CHARGE_AREA_WIDTH       18
#define CHARGE_AREA_HEIGHT      6
#define u8g_logo_width 42
#define u8g_logo_height 55
static const unsigned char PROGMEM med_logo[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 
  0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 
  0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x01, 0xFE, 0xFF, 0x03, 
  0xFF, 0x3F, 0x00, 0xF0, 0xFF, 0x03, 0xFF, 0x1F, 0x00, 0xE0, 0xFF, 0x03, 
  0xFF, 0x0F, 0xFE, 0xC1, 0xFF, 0x03, 0xFF, 0x87, 0xFF, 0x87, 0xFF, 0x03, 
  0xFF, 0xC3, 0xFF, 0x0F, 0xFF, 0x03, 0xFF, 0xE3, 0xFF, 0x1F, 0xFF, 0x03, 
  0xFF, 0xE1, 0xFF, 0x1F, 0xFE, 0x03, 0xFF, 0xF1, 0xFF, 0x3F, 0xFE, 0x03, 
  0xFF, 0xF1, 0xFF, 0x3F, 0xFE, 0x03, 0xFF, 0xF8, 0xFF, 0x7F, 0xFC, 0x03, 
  0xFF, 0xF8, 0xFF, 0x7F, 0xFC, 0x03, 0xFF, 0xF8, 0xFF, 0x7F, 0xFC, 0x03, 
  0xFF, 0xF8, 0xFF, 0x7F, 0xFC, 0x03, 0xFF, 0xF8, 0xFF, 0x7F, 0xFC, 0x03, 
  0xFF, 0xF1, 0xFF, 0x3F, 0xFE, 0x03, 0xFF, 0xF1, 0xFF, 0x3F, 0xFE, 0x03, 
  0xFF, 0xE1, 0xFF, 0x1F, 0xFE, 0x03, 0xFF, 0xE3, 0xFF, 0x1F, 0xFF, 0x03, 
  0xFF, 0xC3, 0xFF, 0x0F, 0xFF, 0x03, 0xFF, 0x87, 0xFF, 0x87, 0xFF, 0x03, 
  0xFF, 0x0F, 0xFE, 0xC1, 0xFF, 0x03, 0xFF, 0x1F, 0x00, 0xE0, 0xFF, 0x03, 
  0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0x03, 0xFF, 0xFF, 0x01, 0xFE, 0xFF, 0x03, 
  0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 
  0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 
  0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x03, 
  0xFF, 0xFF, 0x01, 0xFE, 0xFF, 0x03, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0x03, 
  0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0x03, 0xFF, 0x3F, 0x00, 0xF0, 0xFF, 0x03, 
  0xFF, 0x1F, 0x00, 0xE0, 0xFF, 0x03, 0xFF, 0x0F, 0x00, 0xC0, 0xFF, 0x03, 
  0xFF, 0x07, 0x00, 0x80, 0xFF, 0x03, 0xFF, 0x03, 0x00, 0x00, 0xFF, 0x03, 
  0xFF, 0x01, 0x00, 0x00, 0xFE, 0x03, 0xFF, 0x00, 0x00, 0x00, 0xFC, 0x03, 
  0x7F, 0x00, 0x00, 0x00, 0xF8, 0x03, 0x3F, 0x00, 0x00, 0x00, 0xF0, 0x03, 
  0x1F, 0x00, 0x00, 0x00, 0xE0, 0x03, 0x0F, 0x00, 0x00, 0x00, 0xC0, 0x03, 
  0x07, 0x00, 0x00, 0x00, 0x80, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void setup() {
  display.firstPage();
    do{
      draw_logo();
    }while(display.nextPage());
    
  status = IMU.begin();
  if (status < 0) {
    check_9dof = false;
  }else{
    check_9dof = true;
  }
  // setting the gyroscope full scale range to +/-500 deg/s / 250 / 500 / 1000 / 2000
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  // setting DLPF bandwidth to 20 Hz // 184 Hz / 92 Hz / 41 Hz / 20 Hz / 10 Hz / 5 Hz /
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
  
  pinMode(chargePlugPin, INPUT);
  pinMode(INTERRUPT_PWR_BUTTON_PIN,INPUT);
  pinMode(INTERRUPT_MODE_BUTTON_PIN,INPUT);
  attachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN), ModeSelect, FALLING);
  
  
    
 /* for(int i=1; i<1000; i++){
    IMU.readSensor();
    gyrox = IMU.getGyroX_rads();
    gyroy = IMU.getGyroY_rads();
    gyroz = IMU.getGyroZ_rads();
    bufferx += gyrox;
    buffery += gyroy;
    bufferz += gyroz;
  }
  errorx = bufferx/1000;
  errory = buffery/1000;
  errorz = bufferz/1000;*/
  delay(500);
}

void loop() {
  delay(20);
  if (check_9dof){
    IMU.readSensor();
    //accelx = IMU.getAccelX_mss();
    //accely = IMU.getAccelY_mss();
    //accelz = IMU.getAccelZ_mss();
    gyrox = IMU.getGyroX_rads();// - errorx;
    gyroy = IMU.getGyroY_rads();// - errory;
    gyroz = IMU.getGyroZ_rads();// - errorz;
    //magx = IMU.getMagX_uT();
    //magy = IMU.getMagY_uT();
    //magz = IMU.getMagZ_uT();
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  QuaternionUpdate(gyrox, gyroy, gyroz);
  quaternion_to_euler(q[0], q[1], q[2], q[3]);
  angle_x = abs(angle_x); angle_y = abs(angle_y); angle_z = abs(angle_z);
  
  if (angle_x >= angle_y && angle_x >= angle_z){
    angle = angle_x;
  }else if (angle_y >= angle_z){
    angle = angle_y;
  }else{
    angle = angle_z;
  }
  
  angle *= 180.0f / PI; // calculated angle ready for showing display
///////////////////////////////////////////////////////////////////////////////////////////
  DividedVoltage = analogRead(BatteryReadPin) * (3.3 / 1023);
  BatteryVoltage += ((DividedVoltage * 1290) / 820); // (Voltage * (R1+R2) / R2)
  BatteryCounter ++;
  if (BatteryCounter >= 75){
    BatteryVoltage /= BatteryCounter;
    BatteryCounter = 0;
    if(BatteryVoltage<4.2 && chargePercent>=100) chargePercent = 100;
    if(BatteryVoltage<4.06 && chargePercent>=90) chargePercent = 90;
    if(BatteryVoltage<3.98 && chargePercent>=80) chargePercent = 80;
    if(BatteryVoltage<3.92 && chargePercent>=70) chargePercent = 70;
    if(BatteryVoltage<3.87 && chargePercent>=60) chargePercent = 60;
    if(BatteryVoltage<3.82 && chargePercent>=50) chargePercent = 50;
    if(BatteryVoltage<3.79 && chargePercent>=40) chargePercent = 40;
    if(BatteryVoltage<3.77 && chargePercent>=30) chargePercent = 30;
    if(BatteryVoltage<3.74 && chargePercent>=20) chargePercent = 20;
    if(BatteryVoltage<3.68 && chargePercent>=10) chargePercent = 10;
    if(BatteryVoltage<3.45 && chargePercent>=5) chargePercent = 5;
    if(BatteryVoltage<3.00 && chargePercent>=0) chargePercent = 0;
    BatteryVoltage = 0.0;
  }
  // charge percent ready for showing display
///////////////////////////////////////////////////////////////////////////////////////////
  display.firstPage();
  do {
    showBatteryLevel(chargePercent, digitalRead(chargePlugPin));
    display.setFont(u8g_font_6x13);
    display.setPrintPos(0,10);
    display.println(F("MEDROCO"));
    
    display.setFont(u8g_font_fur25r);
    if (ButtonCounter == 0){
      
      //display.setFont(u8g_font_unifont);
      display.setPrintPos(8,50);
      display.print(F("READY"));
      
    }else if (ButtonCounter == 1){
      display.drawLine(0, 38, 20, 38);
      display.drawLine(108, 38, 128, 38);
      //display.setFont(u8g_font_unifont);
      if (angle<10){
        display.setPrintPos(42,50);
      }else if(angle>=10 && angle<100){
        display.setPrintPos(30,50);
      }else if(angle>=100){
        display.setPrintPos(18,50);
      }
      display.print(angle,1);
      //display.write(176);//display.println(F("°"));
      angleTemp = angle;
      
    }else{
      display.drawLine(0, 15, 128, 15);
      display.drawLine(0, 60, 128, 60);
      //display.setFont(u8g_font_unifont);
      if (angleTemp<10){
        display.setPrintPos(42,50);
      }else if(angleTemp>=10 && angleTemp<100){
        display.setPrintPos(30,50);
      }else if(angleTemp>=100){
        display.setPrintPos(18,50);
      }
      display.print(angleTemp,1);
      //display.write(176);//display.println(F("°"));
    }
  }while( display.nextPage() );
  
///////////////////////////////////////////////////////////////////////////////////////////
  if(digitalRead(INTERRUPT_PWR_BUTTON_PIN)== HIGH){
    display.firstPage();
    do{
      draw_logo();
    }while(display.nextPage());
    delay(500);
    //display.firstPage();
    
    detachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN));
    attachInterrupt (digitalPinToInterrupt(INTERRUPT_PWR_BUTTON_PIN), toggleSleepState, FALLING);
    ButtonCounter = 0;
    intCounter = 0;
    chargePercent = 100;
    while(digitalRead(INTERRUPT_PWR_BUTTON_PIN) == HIGH){
      intCounter ++;
      delay(25);
      if (intCounter == 20) {
        display.firstPage();
        do{
          
        }while(display.nextPage());
      }
    }
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////////
void QuaternionUpdate(float gx, float gy, float gz){
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float qDot1, qDot2, qDot3, qDot4;

  norm = sqrtf(gx * gx + gy * gy + gz * gz);
  if (norm < 0.055f) return;

  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

  q1 += qDot1 * deltat; // q1 = (0.9615 * (q1 + (qDot1*deltat))) + (0.0385*qDot1)
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;

  /*q1 = (0.9615 * (q1 + (qDot1*deltat))) + (0.0385*qDot1);
  q2 = (0.9615 * (q2 + (qDot2*deltat))) + (0.0385*qDot2);
  q3 = (0.9615 * (q3 + (qDot3*deltat))) + (0.0385*qDot3);
  q4 = (0.9615 * (q4 + (qDot4*deltat))) + (0.0385*qDot4);*/

  
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void quaternion_to_euler(float w, float x, float y, float z){
  float t0, t1, t2, t3, t4, t5;

  t0 = 2 * (w*x + y*z);
  t1 = 1 - 2 * (x*x - z*z);
  angle_x = atan2(t0, t1);

  t2 = 2 * (w*y + z*x);
  t3 = 1 - 2 * (y*y -x*x);
  angle_y = atan2(t2, t3);

  t4 = 2 * (w*z + x*y);
  t5 = 1 - 2 * (z*z - y*y);
  angle_z = atan2(t4, t5);
}
///////////////////////////////////////////////////////////////////////////////////////////
void ModeSelect(){
  
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 500) {
      ButtonCounter ++;
      if (ButtonCounter >=3) ButtonCounter = 0;
    }
  last_interrupt_time = interrupt_time;
  if(ButtonCounter == 0 || ButtonCounter == 1 ){
  q[0] = 1.0f;
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = 0.0f;
  angle = 0.0f;
  }
  /*if(ButtonCounter == 1){
    detachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN));
    attachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN), ModeSelect, RISING);
  }
   if(ButtonCounter == 0){
    detachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN));
    attachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN), ModeSelect, FALLING);
  }*/
}

void toggleSleepState() {

    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 500) 
    {
        // toggle state of sleep
        isSleeping = !isSleeping;

        if (isSleeping == true) {
          display.sleepOn();
            goToSleep();
        }
        else {
            wakeUp();
            display.sleepOff();
        }
    }
    last_interrupt_time = interrupt_time;
}

void goToSleep() {
  static byte prevADCSRA = ADCSRA;
  ADCSRA = 0;

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // the MCU Control Register (MCUCR)
  MCUCR = bit (BODS) | bit (BODSE);
  // The BODS bit is automatically cleared after three clock cycles so we better get on with it
  MCUCR = bit (BODS);

  interrupts();
  // And enter sleep mode as set above
  sleep_cpu();
  sleep_disable();
  ADCSRA = prevADCSRA;
}

void wakeUp () {
  attachInterrupt (digitalPinToInterrupt(INTERRUPT_MODE_BUTTON_PIN), ModeSelect, FALLING);
  detachInterrupt (digitalPinToInterrupt(INTERRUPT_PWR_BUTTON_PIN));
  noInterrupts();
  if (intCounter>=20) asm volatile ("  jmp 0");
  //asm volatile ("  jmp 0");
}
///////////////////////////////////////////////////////////////////////////////////////////
void draw_logo(){
  display.drawXBMP( 43, 5, u8g_logo_width, u8g_logo_height, med_logo);
}
void showBatteryLevel(uint8_t percent, uint8_t charging)
{
    uint8_t width;
    if (percent > 100)
        percent = 100;

    //display.drawBitmap(0, 0, battery_bitmap, 64, 64, SSD1306_WHITE);
    display.drawFrame(CHARGE_AREA_START_X-2, CHARGE_AREA_START_Y-2, CHARGE_AREA_WIDTH+4, CHARGE_AREA_HEIGHT+4);
    
    if (charging)
    {   chargePercent = 100;
        BatteryVoltage = 0;
        BatteryCounter = 0;
        //display.setTextSize(1);
        //display.setTextColor(SSD1306_WHITE);
        display.setFont(u8g_font_6x13);
        display.setPrintPos(CHARGE_AREA_START_X+2, CHARGE_AREA_START_Y+7);
        display.write(171);
        display.setPrintPos(CHARGE_AREA_START_X+11, CHARGE_AREA_START_Y+7);
        display.write(187);
    } else {
        width = (percent * CHARGE_AREA_WIDTH) / 100;
        
        /*display.setFont(u8g_font_7x14B);
        if (percent<30)
          display.drawStr90(CHARGE_AREA_START_X+2,CHARGE_AREA_START_Y, "!");
        else*/
        
        //battery reduce right to left
        display.drawBox(CHARGE_AREA_START_X, CHARGE_AREA_START_Y, width, CHARGE_AREA_HEIGHT);
        //battery reduce left to right
        //u8g.drawRBox(CHARGE_AREA_START_X+(CHARGE_AREA_WIDTH-width), CHARGE_AREA_START_Y, width, CHARGE_AREA_HEIGHT, 2);
    }
}
