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
  // if (i<100) Cone(); else if (i<200) Cube(); else Flash();
  /*
  Flash(i);
  i++;
  
  i%=512;
  */

  /*
  j++;
  if (j>=150) {
    j=0;
    state++;
    state%=3;
  }
  //*/

  switch (state) {
    case 0:
      Circles();
      break;
    case 1:
      Cone();
      break;
    case 2:
      Cube2();
      break;
    default:
      Clear();
      Flush();
      break;
  }
}

void LEDController::SetState(int state) {this->state = state;}
int LEDController::GetState() {return state;}

void LEDController::Clear() {
  for (int x=0; x<32; x++) {
    for (int y=0; y<16; y++) {
      SetRGB(x, y, 0, 0, 0);
    }
  }
  //Flush();
}

void LEDController::Circles() {
  Clear();
  for (int x=0; x<16; x++) {
    for (int y=0; y<16; y++) {
      double d = sqrt((x-8)*(x-8) + (y-8)*(y-8));
      SetRGB(x,y, 10+10*sin((i/200.0+d/10.0)*2*std::numbers::pi), 0, 0);
    }
  }
  Flush();
  i++;
  i%=200;
}

void LEDController::Cone() {
  Clear();
  for (int x=0; x<16; x++) {
    int maxy = 12-3*abs(x-8);

    // Triangle of cone
    if (maxy>0) {
      for (int y=0; y<=maxy; y++) {
        SetRGB(x, y, 25, 7, 0);
      }
    }

    // Cool stripes
    for (int y=std::max(0,maxy+1); y<16; y++) {
      double d = y-maxy;
      double s = sin((i/100.0-d/10.0)*2*std::numbers::pi);
      SetRGB(x, y, 6+6*s, 1+1*s, 0);
    }

    // Base of cone
    if (x>=2 && x<14) {
      for (int y=0; y<2; y++) {
        SetRGB(x, y, 25, 7, 0);
      }
    }
  }
  Flush();
  i++;
  i%=100;
}

void LEDController::Cube() {
  Clear();
  for (int y = 0; y<16; y++) {
    int minx = 4;
    int maxx = 11;
    if (y == 0||y == 15) {
      minx = 5;
      maxx = 10;
    }

    for (int x=minx; x<=maxx; x++) {
      SetRGB(x, y, 11, 0, 25);
    }
  }
  Flush();
}

void LEDController::Cube2() {
  Clear();
  for (int x = 0; x<16; x++) {
    for (int y = 0; y<16; y++) {
      if (x<3 || x>12 || y<3 || y>12) {
        double a = atan2(y-8,x-8);
        double s = sin((i/75.0-a/std::numbers::pi)*2*std::numbers::pi);
        double s2 = sin((i/75.0-a/std::numbers::pi)*std::numbers::pi + std::numbers::pi/4);
        SetRGB(x, y, 6*pow(s2,10), 0, 5+5*s);
      }
    }
  }

  for (int y = 4; y<12; y++) {
    for (int x = 4; x<12; x++) {
      if (((x-8)*(x-8)+(y-8)*(y-8))<25)
        SetRGB(x, y, 11, 0, 25);
    }
  }

  /*
  SetRGB(9, 1, 1, 0, 2);
  SetRGB(9, 14, 1, 0, 2);
  for (int x=10;x<22;x++) {
    SetRGB(x, 2, 1, 0, 2);
    SetRGB(x, 13, 1, 0, 2);
  }
  for (int x=12;x<20;x++) {
    SetRGB(x, 3, 1, 0, 2);
    SetRGB(x, 2, 11, 0, 25);
    
    SetRGB(x, 12, 1, 0, 2);
    SetRGB(x, 13, 11, 0, 25);
  }
  SetRGB(22, 1, 1, 0, 2);
  SetRGB(22, 14, 1, 0, 2);
  */

  Flush();

  i++;
  i%=75;
}

void LEDController::Flash(int i) {
  Clear();

  
  for(int h=0; h < i; h++){
    int x = h % 32;
    int y = h / 32;
    
    // int blink = (i%2)*255;
    // SetRGB(0, 0, blink, blink, blink);  
    
    
    SetRGB(x, y, x+1, 0, i);
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
  double i = index%kLength;
  m_ledBuffer[i<0?i+kLength:i].SetRGB(r, g, b);
}

void LEDController::SetRGB(int x, int y, int r, int g, int b) {
  SetRGB(pos(x, y), r, g, b);
}

void LEDController::SetHSV(int index, int h, int s, int v) {
  double i = index%kLength;
  m_ledBuffer[i<0?i+kLength:i].SetHSV(h, s, v);
}

void LEDController::SetHSV(int x, int y, int h, int s, int v) {
  SetHSV(pos(x, y), h, s, v);
}

void LEDController::Flush() {
  m_led.SetData(m_ledBuffer);
}