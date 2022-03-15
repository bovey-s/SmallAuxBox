#include <Arduino.h>
#include <SPI.h>
#include <LinkedList.h>
#include "Adafruit_ILI9341_waveshare.h"
#include "ADS12xx.h"
ADS12xx adc;

#define ADC1_CS       9
#define ADC1_DRDY     8
#define ADC1_RST      7
#define ADC1_START    10


//for pcb version
#define TFT_CS  6
#define TFT_DC  5
#define TFT_RST 4
#define TFT_BL  3

#define ADC1        1
#define LCD         2


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
#define NUM_CHIPS   2
int cs_pins[] = {ADC1_CS, TFT_CS};
  
//#define PGA  1.0             
#define VREF 2.5
//#define VFSR (5/PGA)/1.175
#define VSOURCE  2.5
#define FSR (((long int)1<<23)-1)
#define Boundary1 43.0

#define R_BIAS      1800

#define MIN_CAL_SAMPLE_TIME     1000
#define NORMAL_SAMPLE_TIME      140

#define MAX_BUF_SIZE    20480 / 8           // max volatile storage 32K, use 20K for buffers     
  
#define POKE              '0'
#define OK                '1'
#define SETM              '4'
#define SETB              '5'
#define SETS              '6'
#define VERIFY            '7'
#define DONE              '\n'

#define DEG2RAD 0.0174532925



byte * send_buf;
double out;
double sensor;
double ref;
double outputs[8];

long cur_time;

#define MEASNUM   7
double resistance [MEASNUM];
double avgresistance;
double oldResistance;


//properties of light ring
int centerx = 150;
int centery = 100;
int radius = 85;
double ring_sensitivity = 1;

#define MESOMAT_ORANGE 0xFB60
#define GREEN_RANGE 0x46A3
#define YELLOW_RANGE 0xE6C5
#define ORANGE_RANGE 0xF424
#define RED_RANGE 0xE841
//Variables for rainflow analysis
LinkedList<float> points = LinkedList<float>();
LinkedList<float> maxbuffer = LinkedList<float>();
LinkedList<float> minbuffer = LinkedList<float>();

float fractional_health = 0;
bool dataFlg = true;
int newmaxFlg = 0;
int normalization_factor = 1;
bool normFlg = true;

bool backgroundFLG=1;

//refresh rate
long lastRefresh;
long refreshTime = 70;

void select_chip( int selected ) {

    for (int i=0; i<NUM_CHIPS; i++)
        if (i==selected-1) 
            digitalWrite(cs_pins[i], LOW);
        else
            digitalWrite(cs_pins[i], HIGH);        
}

void setup() {
  
    Serial.begin(115200);

    digitalWrite(TFT_RST,HIGH);
    digitalWrite(TFT_BL,HIGH);
    select_chip(LCD);
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(0);
    tft.invertDisplay(1);
    pinMode(2, INPUT);

    
    select_chip(ADC1);
    adc.begin(ADC1_CS,ADC1_DRDY, ADC1_RST, DR_100);
    //adc.begin()
    //digitalWrite(ADC1_RST,HIGH);
    //adc.Reset();
    
    delay(10);
    //adc.SetRegisterValue(OFC0,50); //set calibrated offset register Value 
    adc.writeRegister(0x00,0x32);

    select_chip(LCD);
    //powerErrorMessage();
    tft.fillScreen(ILI9341_BLACK);
    lastRefresh = millis();
  
    //adc.SetRegisterValue(SYS0,DOR3_5);//set data rate to 40
    //Serial.println("finished setup");
} 

void drawlogo(int x, int y){
  tft.fillCircle(x,y,40,MESOMAT_ORANGE);
  //draw long arms
    tft.drawLine(x - 40, y -1 , x + 61, y -21, MESOMAT_ORANGE);
    tft.drawLine(x - 40, y -2 , x + 61, y -22, MESOMAT_ORANGE);
    tft.drawLine(x - 40, y -3 , x + 61, y -23, MESOMAT_ORANGE);
    tft.drawLine(x - 40, y -4 , x + 61, y - 24, MESOMAT_ORANGE);
    tft.drawLine(x - 40, y -5 , x + 61, y -25, MESOMAT_ORANGE);
//raise small side y by 4

    tft.drawLine(x - 61, y + 23 , x + 40, y +3, MESOMAT_ORANGE);
    tft.drawLine(x - 61, y + 22 , x + 40, y +2, MESOMAT_ORANGE);
    tft.drawLine(x - 61, y + 21 , x + 40, y +1, MESOMAT_ORANGE);
    tft.drawLine(x - 61, y + 20 , x + 40, y +0, MESOMAT_ORANGE);
    tft.drawLine(x - 61, y + 19 , x + 40, y -1, MESOMAT_ORANGE);
  
  for (int i=0; i<4; i++){
    tft.drawLine(x - 40, y - 6 + 10*i, x + 40, y -22 + 10*i, ILI9341_BLACK);
    tft.drawLine(x - 40, y - 7 + 10*i, x + 40, y -23 + 10*i, ILI9341_BLACK);
    tft.drawLine(x - 40, y - 8 + 10*i, x + 40, y -24 + 10*i, ILI9341_BLACK);
    tft.drawLine(x - 40, y - 9 + 10*i, x + 40, y -25 + 10*i, ILI9341_BLACK);
    tft.drawLine(x - 40, y - 10 + 10*i, x + 40, y -26 + 10*i, ILI9341_BLACK);
  }

  
}

void drawBackground(){
  //this isn't called every frame for speed reasons, only once in setup()
 tft.fillScreen(ILI9341_BLACK);
 
 tft.drawCircle(centerx,centery,radius+2,ILI9341_RED);
 tft.drawCircle(centerx,centery,radius+3,ILI9341_RED);

//Rectangle 1
 tft.fillRect(29,13, 17,175,ILI9341_RED);
 tft.fillRect(30,14, 15,173,ILI9341_BLACK);

//Rectangle 2
 //tft.fillRect(5,4,17,233,ILI9341_RED);
 //tft.fillRect(6,5,15,231,ILI9341_BLACK);

 for (int i = MEASNUM -1; i >= 0; i--){
    resistance[i]= -10;
 }
  tft.setCursor(30, 220);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.print("R =");
  tft.setCursor(30,195);
  tft.print("H =");
  drawlogo(151,100);
 
}

void powerErrorMessage(){
  tft.fillScreen(ILI9341_BLACK);
  drawlogo(151,70);

  tft.setCursor(0, 110);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2.5);
  tft.print("Error: Please unplug and\nre-connect power cable\nbefore connecting USB\ncable");
  
}

void fillArc(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{

  byte seg = 3; // Segments are 3 degrees wide = 120 segments for 360 degrees
  byte inc = 3; // Draw segments every 3 degrees, increase to 6 for segmented ring

  // Draw colour blocks every inc degrees
  for (int i = start_angle; i < start_angle + seg * seg_count; i += inc) {
    // Calculate pair of coordinates for segment start
    double sx = cos((i - 90) * DEG2RAD);
    double sy = sin((i - 90) * DEG2RAD);
    uint16_t x0 = sx * (rx - w) + x;
    uint16_t y0 = sy * (ry - w) + y;
    uint16_t x1 = sx * rx + x;
    uint16_t y1 = sy * ry + y;

    // Calculate pair of coordinates for segment end
    double sx2 = cos((i + seg - 90) * DEG2RAD);
    double sy2 = sin((i + seg - 90) * DEG2RAD);
    int x2 = sx2 * (rx - w) + x;
    int y2 = sy2 * (ry - w) + y;
    int x3 = sx2 * rx + x;
    int y3 = sy2 * ry + y;

    tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
  }
}

void drawResistance(){
  select_chip(LCD);
  if (backgroundFLG == 1){
        backgroundFLG = 0;
        drawBackground();
      }
  //draws the resistance on the screen.
  int omitted = 0;
  avgresistance = 0;
  
  for (int i = MEASNUM -1; i > 0; i--){
    resistance[i] = resistance[i-1];
    if (resistance[i] != -10){
      avgresistance += resistance[i];
    }else{
      omitted += 1;
    }
  } 
  
  resistance[0] = abs(outputs[0]);
    if (resistance[0] != -10){
      avgresistance += resistance[0];
    }else{
      omitted += 1;
    }
  avgresistance = avgresistance / (MEASNUM-omitted);

  //int fillArc(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour);
  fillArc( centerx,centery, (int)(oldResistance) % 360 ,6,radius,radius,8,ILI9341_BLACK);
  if (avgresistance <100000)
    fillArc( centerx,centery, (int)(ring_sensitivity * avgresistance) % 360,6,radius,radius,8,ILI9341_WHITE);
  
   //now also update resistance labels
   tft.fillRect(100,195, 320,45,ILI9341_BLACK);
   tft.setCursor(100, 220);
   tft.setTextColor(ILI9341_WHITE);
   tft.setTextSize(3);
   //tft.print(avgresistance,2);
   
   if (avgresistance < 100000){
      tft.print(avgresistance,2);
      tft.print(" Ohms");    
   } else {
      tft.print("Disconnected");
   }

   tft.setCursor(100, 195);
   if (fractional_health*100 > 100)
   {
    tft.setTextSize(2);
    tft.print("Above Max Strain");
   }
   else{
    tft.print(fractional_health*100);
    tft.print(" %");
   }
   int height = round(fractional_health*(172))+1;
   if (fractional_health*100 > 100)
    height = 173;
   if (fractional_health*100 < 50)
    tft.fillRect(30,14, 15,height,GREEN_RANGE);
   else if (fractional_health*100 < 75)
    tft.fillRect(30,14, 15,height,YELLOW_RANGE);
   else if (fractional_health*100 < 90)
    tft.fillRect(30,14, 15,height,ORANGE_RANGE);
   else if (fractional_health*100 < 100)
    tft.fillRect(30,14, 15,height,RED_RANGE);
    else
      tft.fillRect(30,14, 15,height,ILI9341_WHITE);

   if (digitalRead(2) != 1)
   {
    tft.fillRect(30,14, 15,173,ILI9341_BLACK);
   }

  /*int height = round(fractional_health*(230))+1;
   if (fractional_health*100 > 100)
    height = 231;
   if (fractional_health*100 < 50)
    tft.fillRect(6,5, 15,height,GREEN_RANGE);
   else if (fractional_health*100 < 75)
    tft.fillRect(6,5, 15,height,YELLOW_RANGE);
   else if (fractional_health*100 < 90)
    tft.fillRect(6,5, 15,height,ORANGE_RANGE);
   else
    tft.fillRect(6,5, 15,height,RED_RANGE);

   if (digitalRead(2) != 1)
   {
    tft.fillRect(6,5,15,231,ILI9341_BLACK);
   }*/
  
   oldResistance = ring_sensitivity * avgresistance;
 
}



void measure(double resistances[]) {

  
//adc.SetRegisterValue(SYS0,DOR3_160);//set data rate
//adc.SetRegisterValue(MUX0,MUX_SP2_AIN0|MUX_SN2_AIN1); //set channels
//adc.SetRegisterValue(MUX0,MUX_SP2_AIN3|MUX_SN2_AIN2); //set channels
while(digitalRead(ADC1_DRDY) > 0){
  continue;
  Serial.println("waiting..");
}

  int32_t adc_data =(int32_t) adc.getConversion(false);

  
//  if (0.0 < resistances[0] <= 43.0){
//    adc.setPGA(1);
//    PGA = 2;
//  }
//  else if (resistances[0] > 43.0){
//    adc.setPGA(0);
//    PGA = 1;
//  }
//  Serial.println(adc.readRegister(0x02),BIN);
//  Serial.println(adc_data); 
  double PGA;
  PGA = 1;
//draw PGA values from the register somehow.... also figure out what the heck the current PGA value is
  if (adc.readRegister(0x02) == 38){
    PGA = 64;
  }
  else if (adc.readRegister(0x02) == 37){
    PGA = 32;
  }
  else if (adc.readRegister(0x02) == 36){
    PGA = 16;
  }
  else if (adc.readRegister(0x02) == 35) {
    PGA = 8;
  }
  else if (adc.readRegister(0x02) == 34) {
    PGA = 4;
  }
  else if (adc.readRegister(0x02) == 33) {
    PGA = 2;
  }
  else {
    PGA = 1;
  }
  //Serial.println(PGA);
  double V0 = (double)(((double)adc_data*((5/PGA)/1.175)*1000.0)/FSR);   
  //Serial.println(V0);
  resistances[0] =2.0* V0 * (R_BIAS) / (VSOURCE*1000.0 - V0);
  //Serial.println(resistances[0]);

  if (resistances[0] >= 2206){
    adc.setPGA(0);
  }
  else if (resistances[0] <= 38) {
  adc.setPGA(6);
  }
  else if (resistances[0] >=  844) {
  adc.setPGA(1);
  }
  else if (resistances[0] <= 79) {
  adc.setPGA(5);
  }
  else if (resistances[0] >= 378) {
  adc.setPGA(2);
  }
  else if (resistances[0] <= 162) {
  adc.setPGA(4);
  }
  else if (resistances[0] >= 179) {
  adc.setPGA(3);
  }
  else if (resistances[0] <= 340) {
  adc.setPGA(3);
  }
  else if (resistances[0] >= 88) {
  adc.setPGA(4);
  }
  else if (resistances[0] <= 760) {
  adc.setPGA(2);
  }
  else if (resistances[0] >= 43) {
  adc.setPGA(5);
  }
  else if (resistances[0] <= 1986) {
  adc.setPGA(1);
  }
  







  
  //Serial.println(adc.readRegister(0x02),BIN); 
  //Serial.println(adc.readRegister(0x02));
  //Serial.println(PGA);
  

}

int getNewExtrema(float input){

  points.add(input);
  if (points.size() < 3)
    return 0;
  float deltaOld = points.get(points.size() - 2) - points.get(points.size() - 3);
  float deltaNew = points.get(points.size() - 1) - points.get(points.size() - 2);
  if (deltaOld * deltaNew > 0){
    points.remove(0);
    return 0;
  }
  else if (deltaNew*100 > 0.022)
  {
    minbuffer.add(points.get(points.size() - 2));
    points.remove(0);
    return 1;
  }
  else if (deltaNew*100 < -0.022)
  {
    maxbuffer.add(points.get(points.size() - 2));
    points.remove(0);
    return 2;
  }
  else{
    points.remove(0);
    return 0;
  }
}

float SNfunc(float input){
            return (float)( pow((double)input, -1.0 / 0.7) / 1.10);
}

bool MaxRFCount(){
            float amplitude;
            float mean;
            float value;
            //return whether new point is required
            if ((maxbuffer.size() < 2) || minbuffer.size() < 1)
                return true;

            if (maxbuffer.get(maxbuffer.size() - 1) < maxbuffer.get(maxbuffer.size()-2)){
                return true;
            }
            if(minbuffer.size() == 1)
            {
                amplitude = abs(maxbuffer.get(maxbuffer.size() - 2) - minbuffer.get(minbuffer.size() - 1));
                mean = (maxbuffer.get(maxbuffer.size() - 2) + minbuffer.get(minbuffer.size() - 1)) / 2;
                value = (float)0.5;
                fractional_health = fractional_health + value / SNfunc(amplitude);
                maxbuffer.remove(maxbuffer.size()-2); //remove last 2 values
                if (maxbuffer.size() > 1)
                {
                    return false;
                }
                else
                {
                    return true;
                }

            }
            else
            {
                amplitude = abs(maxbuffer.get(maxbuffer.size() - 2) - minbuffer.get(minbuffer.size() - 1));
                mean = (maxbuffer.get(maxbuffer.size() - 2) + minbuffer.get(minbuffer.size() - 1)) / 2;
                value = (float)1;
                fractional_health = fractional_health + value / SNfunc(amplitude);
                maxbuffer.remove(maxbuffer.size() - 2);
                minbuffer.remove(minbuffer.size()-1);
                if (maxbuffer.size() > 1)
                {
                    return false;
                }
                else
                {
                    return true;
                }

            }
}

bool MinRFCount(){
            float amplitude;
            float mean;
            float value;
            //return whether new point is required
            if ((maxbuffer.size() < 1) || minbuffer.size() < 2)
                return true;

            if (minbuffer.get(minbuffer.size() - 1) > minbuffer.get(minbuffer.size() - 2))
            {
                return true;
            }
            if (maxbuffer.size() == 1)
            {
                amplitude = abs(maxbuffer.get(maxbuffer.size() - 1) - minbuffer.get(minbuffer.size() - 2));
                mean = (maxbuffer.get(maxbuffer.size() - 1) + minbuffer.get(minbuffer.size() - 2)) / 2;
                value = (float)0.5;
                fractional_health = fractional_health + value / SNfunc(amplitude);
                minbuffer.remove(minbuffer.size() - 2);
                if (minbuffer.size() > 1)
                {
                    return false;
                }
                else
                {
                    return true;
                }

            }
            else
            {
                amplitude = abs(maxbuffer.get(maxbuffer.size() - 1) - minbuffer.get(minbuffer.size() - 2));
                mean = (maxbuffer.get(maxbuffer.size() - 1) + minbuffer.get(minbuffer.size() - 2)) / 2;
                value = (float)1;
                fractional_health = fractional_health + value / SNfunc(amplitude);
                maxbuffer.remove(maxbuffer.size()-1);
                minbuffer.remove(minbuffer.size() - 2);
                if (minbuffer.size() > 1)
                {
                    return false;
                }
                else
                {
                    return true;
                }

            }
}

void loop() { 
  int settlingTime = 21;
  long adcTime = micros();
  if (dataFlg){
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 
    select_chip(ADC1);
    //Serial.println("measuring");
    while ((micros()-adcTime)<settlingTime){
      adcTime = micros();
      measure(outputs);
    }
    //Serial.println("measured");
    if ((millis()-lastRefresh) > refreshTime){
      drawResistance();
      //drawMatHealth();
      lastRefresh = millis();
    }
  
    if (normFlg == true){
      normalization_factor = outputs[0] * 0.8;
      normFlg = false;
    }
    
    newmaxFlg = getNewExtrema(outputs[0] / normalization_factor);
  
    SPI.endTransaction();
  }
  //Serial.println(digitalRead(2));
  //Serial.println(maxbuffer.get(1));
  if (newmaxFlg == 2)
  {
      dataFlg = MaxRFCount();
  }
      
  if (newmaxFlg == 1)
  {
      dataFlg = MinRFCount();
  }
  if (digitalRead(2) != 1){
    fractional_health = 0;
    points.clear();
    maxbuffer.clear();
    minbuffer.clear();
  }
      if (Serial.available() > 0) 
      {
          String contents = Serial.readStringUntil(10);
          char code = '0';
          int value;
          double value_f;
          if (contents.length() > 0)
          {
            code = contents[0];
            if (contents.length() > 1)
            {
              value = contents.substring(1).toInt();
              value_f = (double)contents.substring(1).toFloat();
            }
          }

          
          switch (code) 
          {
              case POKE:
                  {
                  // write adc data to reference and sensor variables
                  //Serial.println(outputs[0],4);
                  double timeOffset = 0;
                  double oldTime = 0;
                  double currentTime = 0;

                  if (currentTime < oldTime)
                    timeOffset += oldTime;
                  currentTime = micros() + timeOffset;
                  oldTime = currentTime;
                  for (int j = 0; j<7; j++){
                    outputs[j+1] = -1.0;
                  }
                   
                  for (int i=0; i<8;i++){
                    Serial.print(String(outputs[i],4));
                    Serial.print(',');
                  }
                  //Serial.println(String(currentTime/1000000,6)); 

                  Serial.print(adc.readRegister(0x02),BIN);
                  }
                  break;

              case VERIFY:
                  {
                  Serial.write(OK);
                  }
                  break;
                  
              case SETS:
                  {
                  ring_sensitivity = (double)value_f;
                  //Serial.println(OK);
                  }
                  break;
                  
              default:
                  {}
                  break;
          }

          
          
      }
delay(1);
}