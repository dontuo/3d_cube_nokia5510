#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
// simple project with rotating 3D cube using Arduino UNO and Transparent 128x64 OLED Display, 
// created by upir, 2022
// youtube channel: https://www.youtube.com/upir_upir
// full tutoral is here: https://youtu.be/kBAcaA7NAlA

// Turbo pressure gauge tutorial: https://youtu.be/JXmw1xOlBdk
// Transparent OLED tutorial: https://youtu.be/hIFDcksXgBk
// Knob + OLED tutorial: https://youtu.be/NPfaLKKsf_Q

// useful links:
// u8g documentation: https://github.com/olikraus/u8glib/wiki/userreference
// Wokwi starting project: https://wokwi.com/arduino/projects/300867986768527882
// Arduino UNO: http://store.arduino.cc/products/arduino-uno-rev3
// Arduino UNO MINI: https://store.arduino.cc/products/uno-mini-le
// Multidimensional arrays: https://www.tutorialspoint.com/arduino/arduino_multi_dimensional_arrays.htm
// 2D Rotation: https://en.wikipedia.org/wiki/Rotation_(mathematics)
// Normal OLED Display: https://www.aliexpress.com/item/4001051535838.html
// Transparent OLED Display: https://a.aliexpress.com/_mKGmhKg
// Big OLED Display: https://www.aliexpress.com/item/1005003091769556.html
// Arduino breadboard prototyping shield: https://www.adafruit.com/product/2077



//#include "U8glib.h" // u8g library, note there is a newer version u8g2, please use the older one

/*const uint8_t upir_logo[] U8G_PROGMEM = {        // another simple way how to define pictures for u8g library
B00010101, B11010111,     //  ░░░█░█░███░█░███
B00010101, B01000101,     //  ░░░█░█░█░█░░░█░█
B00010101, B10010110,     //  ░░░█░█░██░░█░██░
B00011001, B00010101      //  ░░░██░░█░░░█░█░█
};*/

// uncomment the correct connection - fast I2C, slow I2C, SPI
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); // Fast I2C / TWI
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK); // slow I2C / TWI     -- I had to use "slow I2C" in my case
//U8GLIB_SSD1306_128X64 u8g(13, 11, 8, 9, 10); // SPI connection  - SCL = 13, SDA = 11, RES = 10, DC = 9, CS = 8
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);
int points[8][2]; // eight 2D points for the cube, values will be calculated in the code
int change_axis;
int orig_points [8][3] = {  // eight 3D points - set values for 3D cube
{-1,-1, 1},
{1,-1,1},
{1,1,1},
{-1,1,1},
{-1,-1,-1},
{1,-1,-1},
{1,1,-1},
{-1,1,-1}
};

float rotated_3d_points [8][3];   // eight 3D points - rotated around Y axis
float angle_deg = 20.0;           // rotation around the Y axis
float z_offset = -4.0;            // offset on Z axis
float cube_size = 60.0;           // cube size (multiplier)
float time_frame;                 // ever increasing time value
float angle_deg2 = 20.0;

void setup() {
  Serial.begin(9600);
  display.begin();
  display.setContrast(50);
  display.clearDisplay();
  display.display();
  pinMode(8, INPUT_PULLUP);
}

void loop() {
  byte button = digitalRead(8);
  
  int poten = analogRead(0);
  poten = map(poten, 0, 1024, 0,180);
  angle_deg = poten;
  //time_frame++;                                // increase the time frame value by 1
  //cube_size = 50 + sin(time_frame * 0.2)*20;   // oscilate cube size between values 30 - 70

  //z_offset =  -2.0;    // 
  //cube_size = 18.0;    // uncomment those two lines for a "wide angle camera" -- bigger perspective distort

  // increase the angle by 5° increments
  if (angle_deg2 < 90-5) {
    angle_deg2 = angle_deg2 + 5;
  } else {
    angle_deg2 = 0;
  }

  // calculate the points
  if (!button){change_axis ++; delay(50);}
  Serial.println(change_axis);
  for (int i=0; i<8; i++) {
      if (change_axis == 1){
      rotated_3d_points [i][0] = orig_points[i][0] * cos(radians(angle_deg)) + orig_points[i][1]* -sin(radians(angle_deg));
      rotated_3d_points [i][1] = orig_points[i][0] * sin(radians(angle_deg)) + orig_points[i][1] * cos(radians(angle_deg));
      rotated_3d_points [i][2] = orig_points[i][2] - z_offset;}
      else if(change_axis == 2){
      rotated_3d_points [i][0] = orig_points [i][0] * cos(radians(angle_deg)) - orig_points [i][2] * sin(radians(angle_deg));
      rotated_3d_points [i][1] = orig_points [i][1];
      rotated_3d_points [i][2] = orig_points [i][0] * sin(radians(angle_deg)) + orig_points [i][2] * cos(radians(angle_deg)) + z_offset;}
      else if(change_axis == 3){
      rotated_3d_points [i][2] = orig_points[i][1] * sin(radians(angle_deg)) + orig_points[i][2] * cos(radians(angle_deg)) - z_offset;
      rotated_3d_points [i][1] = orig_points[i][1] * cos(radians(angle_deg)) + orig_points[i][2] * -sin(radians(angle_deg));
      rotated_3d_points [i][0] = orig_points[i][0];}
      else if(change_axis >= 4){ change_axis = 0;}
    // rotate 3d points around the z axis (rotating x and y positions)
    /*rotated_3d_points [i][0] = orig_points[i][0] * cos(radians(angle_deg)) + orig_points[i][1]* -sin(radians(angle_deg));
    rotated_3d_points [i][1] = orig_points[i][0] * sin(radians(angle_deg)) + orig_points[i][1] * cos(radians(angle_deg));
    rotated_3d_points [i][2] = orig_points[i][2] - z_offset;*/
    // rotate 3d points around the Y axis (rotating X nad Z positions)
    /*rotated_3d_points [i][0] = orig_points [i][0] * cos(radians(angle_deg)) - orig_points [i][2] * sin(radians(angle_deg));
    rotated_3d_points [i][1] = orig_points [i][1];
    rotated_3d_points [i][2] = orig_points [i][0] * sin(radians(angle_deg)) + orig_points [i][2] * cos(radians(angle_deg)) + z_offset;*/
    //rotate 3d points around x axis (rotating y and z positions)
    /*rotated_3d_points [i][2] = orig_points[i][1] * sin(radians(angle_deg)) + orig_points[i][2] * cos(radians(angle_deg)) - z_offset;
    rotated_3d_points [i][1] = orig_points[i][1] * cos(radians(angle_deg)) + orig_points[i][2] * -sin(radians(angle_deg));
    rotated_3d_points [i][0] = orig_points[i][0];*/
    // project 3d points into 2d space with perspective divide -- 2D x = x/z,   2D y = y/z
    points[i][0] = round(84/2 + rotated_3d_points [i][0] / rotated_3d_points [i][2] * cube_size);
    points[i][1] = round(48/2 + rotated_3d_points [i][1] / rotated_3d_points [i][2] * cube_size);    
    }

  
      // connect the lines between the individual points
      display.drawLine(points[ 0 ][ 0 ], points[ 0 ][ 1 ] , points[ 1 ][ 0 ] , points[ 1 ][ 1 ],1 );  // connect points 0-1
      display.drawLine(points[ 1 ][ 0 ], points[ 1 ][ 1 ] , points[ 2 ][ 0 ] , points[ 2 ][ 1 ],1 );  // connect points 1-2  
      display.drawLine(points[ 2 ][ 0 ], points[ 2 ][ 1 ] , points[ 3 ][ 0 ] , points[ 3 ][ 1 ],1 );  // connect points 2-3      
      display.drawLine(points[ 3 ][ 0 ], points[ 3 ][ 1 ] , points[ 0 ][ 0 ] , points[ 0 ][ 1 ],1 );  // connect points 3-0      

      display.drawLine(points[ 4 ][ 0 ], points[ 4 ][ 1 ] , points[ 5 ][ 0 ] , points[ 5 ][ 1 ],1 );  // connect points 4-5
      display.drawLine(points[ 5 ][ 0 ], points[ 5 ][ 1 ] , points[ 6 ][ 0 ] , points[ 6 ][ 1 ],1 );  // connect points 5-6  
      display.drawLine(points[ 6 ][ 0 ], points[ 6 ][ 1 ] , points[ 7 ][ 0 ] , points[ 7 ][ 1 ],1 );  // connect points 6-7      
      display.drawLine(points[ 7 ][ 0 ], points[ 7 ][ 1 ] , points[ 4 ][ 0 ] , points[ 4 ][ 1 ],1 );  // connect points 7-4  

      display.drawLine(points[ 0 ][ 0 ], points[ 0 ][ 1 ] , points[ 4 ][ 0 ] , points[ 4 ][ 1 ],1 );  // connect points 0-4
      display.drawLine(points[ 1 ][ 0 ], points[ 1 ][ 1 ] , points[ 5 ][ 0 ] , points[ 5 ][ 1 ],1 );  // connect points 1-5  
      display.drawLine(points[ 2 ][ 0 ], points[ 2 ][ 1 ] , points[ 6 ][ 0 ] , points[ 6 ][ 1 ],1 );  // connect points 2-6      
      display.drawLine(points[ 3 ][ 0 ], points[ 3 ][ 1 ] , points[ 7 ][ 0 ] , points[ 7 ][ 1 ],1 );  // connect points 3-7                 

      display.display();
      //delay(150);
      display.clearDisplay();
      }
