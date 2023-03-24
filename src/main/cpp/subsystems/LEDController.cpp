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
    case 3:
      DrawWord();
      break;
    case 4: // Field-relative (blue)
      Fill(0,0,255);
      DrawLetter('F', 7, 6);
      Flush();
      break;
    case 5: // Robot-relative (red)
      Fill(255,0,0);
      DrawLetter('R', 7, 6);
      Flush();
      break;
    case 6: // Angles (for autobalance)
      DrawAngle();
      break;
    default:
      Clear();
      Flush();
      break;
  }
}

// 0: Hypno
// 1: Cone
// 2: Cube
// 3: Scrolling text
// 4: Field-relative
// 5: Robot-relative
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

void LEDController::Fill(int r, int g, int b) {
  for (int x=0; x<32; x++) {
    for (int y=0; y<16; y++) {
      SetRGB(x, y, r, g, b);
    }
  }
}

void LEDController::Circles() {
  Clear();
  for (int x=0; x<16; x++) {
    for (int y=0; y<16; y++) {
      double d = sqrt((x-8)*(x-8) + (y-8)*(y-8));
      SetRGB(x,y, 80+120*sin((i/200.0+d/10.0)*2*std::numbers::pi), 0, 0);
    }
  }
  Flush();
  i++;
  i%=200;
}

void LEDController::Cone() {
  Clear();
  for (int x=0; x<16; x++) {
    int miny = 3*abs(x-8)-4;

    // Triangle of cone
    if (miny>0) {
      for (int y=miny; y<16; y++) {
        SetRGB(x, y, 255, 70, 0);
      }
    }

    // Cool stripes
    for (int y=0; y<miny; y++) {
      double d = miny-y;
      double s = sin((i/100.0-d/10.0)*2*std::numbers::pi);
      SetRGB(x, y, 60+60*s, 10+10*s, 0);
    }

    // Base of cone
    if (x>=2 && x<14) {
      for (int y=14; y<16; y++) {
        SetRGB(x, y, 255, 70, 0);
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
      SetRGB(x, y, 110, 0, 255);
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
        SetRGB(x, y, 60*pow(s2,10), 0, 50+50*s);
      }
    }
  }

  for (int y = 4; y<12; y++) {
    for (int x = 4; x<12; x++) {
      if (((x-8.5)*(x-8.5)+(y-8.5)*(y- 8.5))<20)
        SetRGB(x, y, 110, 0, 255);
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

void LEDController::DrawWord() {
  Clear();
  for (int k=0; k<(int) strlen(word.c_str()); k++) {
    DrawLetter(word[k], 4*k+16-floor(i/4), 6);
  }
  Flush();

  i++;
  i%=(4*strlen(word.c_str())+16)*4;
}

void LEDController::DrawLetter(char c, int x, int y) {
  switch (c) {
    case 'A':
      for (int y2=y+1;y2<y+5;y2++) {
        SetRGB(x,y2,   0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      break;
    case 'B':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y+1, 0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      SetRGB(x+2,y+3, 0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'C':
      for (int y2=y+1;y2<y+4;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      for (int x2=x+1;x2<x+3;x2++) {
        SetRGB(x2,y,   0, 0, 255);
        SetRGB(x2,y+4, 0, 0, 255);
      }
      break;
    case 'D':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y+1, 0, 0, 255);
      SetRGB(x+2,y+2, 0, 0, 255);
      SetRGB(x+2,y+3, 0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'E':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y,   0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      SetRGB(x+2,y+4, 0, 0, 255);
      break;
    case 'F':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y,   0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      break;
    case 'G':
      for (int y2=y+1;y2<y+4;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      for (int x2=x+1;x2<x+3;x2++) {
        SetRGB(x2,y,   0, 0, 255);
        SetRGB(x2,y+4, 0, 0, 255);
      }
      SetRGB(x+2,y+3, 0, 0, 255);
      SetRGB(x+2,y+2, 0, 0, 255);
      break;
    case 'H':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+2, 0, 0, 255);
      break;
    case 'I':
      SetRGB(x,y,     0, 0, 255);
      SetRGB(x+2,y,   0, 0, 255);
      SetRGB(x,y+4,   0, 0, 255);
      SetRGB(x+2,y+4, 0, 0, 255);
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x+1,y2, 0, 0, 255);
      }
      break;
    case 'J':
      for (int y2=y+1;y2<y+4;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'K':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
        if (y2!=2) SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+2, 0, 0, 255);
      break;
    case 'L':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+4, 0, 0, 255);
      SetRGB(x+2,y+4, 0, 0, 255);
      break;
    case 'M':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2,   0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+1,y+1, 0, 0, 255);
      break;
    case 'N':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2,   0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      break;
    case 'O':
      for (int y2=y+1;y2<y+4;y2++) {
        SetRGB(x,y2,   0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'P':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y+1, 0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      break;
    case 'Q':
      //TODO
      break;
    case 'R':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y+1, 0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      SetRGB(x+2,y+3, 0, 0, 255);
      SetRGB(x+2,y+4, 0, 0, 255);
      break;
    case 'S':
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y,   0, 0, 255);
      SetRGB(x,y+1,   0, 0, 255);
      SetRGB(x,y+2,   0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      SetRGB(x+2,y+2, 0, 0, 255);
      SetRGB(x+2,y+3, 0, 0, 255);
      SetRGB(x,y+4,   0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'T':
      SetRGB(x,y,   0, 0, 255);
      SetRGB(x+2,y, 0, 0, 255);
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x+1,y2, 0, 0, 255);
      }
      break;
    case 'U':
      for (int y2=y;y2<y+4;y2++) {
        SetRGB(x,y2, 0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'V':
      for (int y2=y;y2<y+3;y2++) {
        SetRGB(x,y2, 0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+3, 0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'W':
      for (int y2=y;y2<y+5;y2++) {
        SetRGB(x,y2, 0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+3, 0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      break;
    case 'X':
      for (int y2=y;y2<y+5;y2++) {
        if (y2!=2) SetRGB(x,y2, 0, 0, 255);
        if (y2!=2) SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y+2, 0, 0, 255);
      break;
    case 'Y':
      for (int y2=y;y2<y+2;y2++) {
        SetRGB(x,y2,   0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      for (int y2=y+2;y2<y+5;y2++) {
        SetRGB(x+1,y2, 0, 0, 255);
      }
      break;
    default:
      return;
  }
}

void LEDController::DrawAngle() {
  Clear();
  for (int x = 0; x<16; x++) {
    for (int y = 0; y<16; y++) {
      double a = atan2(y-8,x-8);
      double s = sin((angle/18.0-a/std::numbers::pi)*2*std::numbers::pi);
      double s2 = sin((angle/18.0-a/std::numbers::pi)*std::numbers::pi + std::numbers::pi/4);
      SetRGB(x, y, 60*pow(s2,10), 0, 50+50*s);
    }
  }

  Flush();
}

void LEDController::SetAngle(double angle) {
  this->angle = angle;
}

int LEDController::pos(int x, int y) {
  if (x<0 || y<0 || x>=16 || y>=16) return -1;
  int output = floor(x/2)*32;
  if (x%2==0) {
    output+=y;
  } else {
    output+=31-y;
  }
  return output;
}

void LEDController::SetRGB(int index, int r, int g, int b) {
  if (index<0 || index>=kLength) return;
  double i = index%kLength; // TODO: clean up (not necessary now we have line above)
  m_ledBuffer[i<0?i+kLength:i].SetRGB(r*bright, g*bright, b*bright);
}

void LEDController::SetRGB(int x, int y, int r, int g, int b) {
  SetRGB(pos(x, y), r, g, b);
}

void LEDController::SetHSV(int index, int h, int s, int v) {
  if (index<0 || index>=kLength) return;
  double i = index%kLength;
  m_ledBuffer[i<0?i+kLength:i].SetHSV(h, s, v*bright);
}

void LEDController::SetHSV(int x, int y, int h, int s, int v) {
  SetHSV(pos(x, y), h, s, v);
}

void LEDController::Flush() {
  m_led.SetData(m_ledBuffer);
}