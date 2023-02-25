// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDController.h"

#include "Constants.h"

#include "Logging.hpp"

using namespace DriveConstants;

LEDController::LEDController() {
  // Default to a length of 512, start empty output
  // Length is expensive to set, so only set it once, then just update data
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

void LEDController::Periodic() {
  /*
  for (int x=0; x<32; x++) {
    for (int y=0; y<16; y++) {
      float d = sqrt((x-16)*(x-16) + (y-8)*(y-8));
      SetRGB(x,y, 10+10*sin((i/200.0+d/10.0)*2*std::numbers::pi), 0, 0);
    }
  }
  Flush();
  i++;
  i%=200;
  */
  if (i<100) Cone(); else if (i<200) Cube(); else Cube2();
  i++;
  i%=300;
}

void LEDController::Clear() {
  for (int x=0; x<32; x++) {
    for (int y=0; y<16; y++) {
      SetRGB(x,y,0,0,0);
    }
  }
  //Flush();
}

void LEDController::Cone() {
  Clear();
  for (int y=1; y<=15; y++) {
    int minx = 15-y/3;
    int maxx = 16+y/3;
    if(y >= 14){
      minx = 9;
      maxx = 22;
    }
  
    for (int x=minx; x<=maxx; x++) {
      SetRGB(x,y, 25, 7, 0);
    }
  }
  Flush();
}

void LEDController::Cube() {
  Clear();
  for(int y = 0; y<16; y++){
    int minx = 7;
    int maxx = 22;
    if(y == 0||y == 15){
      minx = 8;
      maxx = 21;
    }
    
    for (int x=minx; x<=maxx; x++) {
        SetRGB(x,y, 11, 0, 25);
    }
  }
  Flush();
}

void LEDController::Cube2() {
  Clear();
  for(int y = 0; y<=15; y++) {
    for(int x = 8; x<=23; x++) {
      SetRGB(x,y, 11, 0, 25);
    }
  }

  SetRGB(8,0, 0,0,0);
  SetRGB(8,1, 0,0,0);
  SetRGB(8,2, 0,0,0);
  SetRGB(9,0, 0,0,0);
  SetRGB(10,0, 0,0,0);

  SetRGB(23,0, 0,0,0);
  SetRGB(23,1, 0,0,0);
  SetRGB(23,2, 0,0,0);
  SetRGB(22,0, 0,0,0);
  SetRGB(21,0, 0,0,0);

  SetRGB(8,15, 0,0,0);
  SetRGB(8,14, 0,0,0);
  SetRGB(8,13, 0,0,0);
  SetRGB(9,15, 0,0,0);
  SetRGB(10,15, 0,0,0);
  
  SetRGB(23,15, 0,0,0);
  SetRGB(23,14, 0,0,0);
  SetRGB(23,13, 0,0,0);
  SetRGB(22,15, 0,0,0);
  SetRGB(21,15, 0,0,0);

  SetRGB(9,1, 1,0,2);
  for(int x=10;x<22;x++){
    SetRGB(x,2, 1,0,2);
  }
  SetRGB(22,1, 1,0,2);
  for(int y=2;y<14;y++){
    SetRGB(21,y, 1,0,2);
  }
  SetRGB(22,14, 1,0,2);
  for(int x=21;x>9;x--){
    SetRGB(x,13, 1,0,2);
  }
  SetRGB(9,14, 1,0,2);
  for(int y=13;y>2;y--){
    SetRGB(10,y, 1,0,2);
  }

  Flush();
}



int LEDController::pos(int x, int y) {
  int output = floor(x/2)*32;
  if (x%2==0) {
    output+=y;
  } else {
    output+=31-y;
  }
  return output;
}

void LEDController::SetRGB(int index, int r, int g, int b) {
  m_ledBuffer[index%kLength].SetRGB(r,g,b);
}

void LEDController::SetRGB(int x, int y, int r, int g, int b) {
  SetRGB(pos(x,y), r,g,b);
}

void LEDController::SetHSV(int index, int h, int s, int v) {
  m_ledBuffer[index%kLength].SetHSV(h,s,v);
}

void LEDController::SetHSV(int x, int y, int h, int s, int v) {
  SetHSV(pos(x,y), h,s,v);
}

void LEDController::Flush() {
  m_led.SetData(m_ledBuffer);
}