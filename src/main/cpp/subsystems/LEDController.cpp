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
  m_led.SetLength(kTotalLength);
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

  switch (overrideState==-1?state:overrideState) {
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
      Pulse(0,0,255,20);
      DrawLetter('F', 7, 6);
      Flush();
      i++; i%=20;
      break;
    case 5: // Robot-relative (red)
      Fill(255,0,0);
      Pulse(255,0,0,20);
      DrawLetter('R', 7, 6);
      Flush();
      i++; i%=20;
      break;
    case 6: // Angles (for autobalance)
      DrawAngle();
      break;
    case 7: // Confirmation (temporary state)
      FlashConfirmation();
      break;
    case 8:
      Aperture();
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
// 6: Draw angle
// 7: Flash Confirmation
// 8: Aperture
void LEDController::SetState(int state) {
  if (this->state!=state) {
    prevState = this->state;
    this->state = state;
    i = 0;
  }
}
int LEDController::GetState() {return state;}
void LEDController::SetOverrideState(int state) {
  if (overrideState!=state) {
    overrideState = state;
    i = 0;
  }
}
int LEDController::GetOverrideState() {return overrideState;}
int LEDController::GetPrevState() {return prevState;}

void LEDController::Clear() {
  for (int x=0; x<32; x++) {
    for (int y=0; y<16; y++) {
      SetRGB(x, y, 0, 0, 0);
    }
  }

  for (int n=0; n < kLength2; n++) {
    SetRGB(n+kLength, 0, 0, 0);
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

void LEDController::Pulse(int r, int g, int b, int loop) {
  for (int n=0; n < kLength2; n++) {
    double s = 0.4 + 0.6*sin((n/10.0 + i*1.0/loop)*2*std::numbers::pi);
    s = std::clamp(s, 0.0, 1.0);
    SetRGB(n+kLength, r*s, g*s, b*s);
  }
}

void LEDController::Circles() {
  Clear();
  for (int x=0; x<16; x++) {
    for (int y=0; y<16; y++) {
      double d = sqrt((x-8)*(x-8) + (y-8)*(y-8));
      double s = 90+110*sin((i/200.0+d/10.0)*2*std::numbers::pi);
      s = std::max(s,0.0);
      if (m_allianceIsRed) {
        SetRGB(x,y, s, 0, 0);
      } else {
        SetRGB(x,y, 0, 0, s);
      }
    }
  }

  if (m_allianceIsRed) Pulse(255,0,0, 20);
  else Pulse(0,0,255, 20);

  Flush();
  i++;
  i%=200;
}

void LEDController::Cone() {
  Clear();
  for (int x=0; x<16; x++) {
    int miny = 3*abs(x-8)+3;

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
    if (x>2 && x<14) {
      for (int y=14; y<16; y++) {
        SetRGB(x, y, 255, 70, 0);
      }
    }
  }

  Pulse(255, 70, 0, 50);

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
      SetRGB(x, y, 110, 0, 255);
    }
  }

  for (int n=0; n < kLength2; n++) {
    double s = sin((i/75.0+n/10.0)*2*std::numbers::pi);
    double s2 = sin((i/75.0+n/10.0)*std::numbers::pi + std::numbers::pi/4);
    SetRGB(n+kLength, 60*pow(s2,10), 0, 50+50*s);
  }

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

  Pulse(0, 0, 255, (4*strlen(word.c_str())+16)*4);

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
      // TODO: Make better Q
      //  X
      // X X
      // X X
      // XXX
      //  XX
      for (int y2=y+1;y2<y+4;y2++) {
        SetRGB(x,y2,   0, 0, 255);
        SetRGB(x+2,y2, 0, 0, 255);
      }
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+1,y+3, 0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      SetRGB(x+3,y+4, 0, 0, 255);
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
    case 'Z':
      SetRGB(x,y,     0, 0, 255);
      SetRGB(x+1,y,   0, 0, 255);
      SetRGB(x+2,y,   0, 0, 255);
      SetRGB(x+2,y+1, 0, 0, 255);
      SetRGB(x+1,y+2, 0, 0, 255);
      SetRGB(x,y+3,   0, 0, 255);
      SetRGB(x,y+4,   0, 0, 255);
      SetRGB(x+1,y+4, 0, 0, 255);
      SetRGB(x+2,y+4, 0, 0, 255);
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
      double s = sin((angle/18.0-2*a/std::numbers::pi)*2*std::numbers::pi);
      double s2 = sin((angle/18.0-2*a/std::numbers::pi)*std::numbers::pi + std::numbers::pi/4);
      SetRGB(x, y, 60*pow(s2,10), 0, 50+50*s);
    }
  }

  for (int n=0; n < kLength2; n++) {
    double s = sin((angle/18.0+n/10.0)*2*std::numbers::pi);
    double s2 = sin((angle/18.0+n/10.0)*std::numbers::pi + std::numbers::pi/4);
    SetRGB(n+kLength, 60*pow(s2,10), 0, 50+50*s);
  }

  Flush();
}

void LEDController::FlashConfirmation() {
  Clear();

  double s = sin(i/25.0 *2*std::numbers::pi)*0.5 + 0.5;

  for (int n = 0; n<kTotalLength; n++) {
    SetRGB(n, 50*s, 205*s, 50*s);
  }

  Flush();

  i++;
  i%=100;
}

void LEDController::Aperture() {
  Clear();

  // LED panel Aperture symbol
  for (unsigned int n = 0; n<aperture_points.size(); n++) {
    int x = aperture_points[n].first;
    int y = aperture_points[n].second;

    SetRGB(x, y,       43, 56, 127); // Upper left
    SetRGB(15-y, x,    43, 56, 127); // Upper right
    SetRGB(15-x, 15-y, 43, 56, 127); // Lower right
    SetRGB(y, 15-x,    43, 56, 127); // Lower left
  }
  
  /*
  for (int n=0; n < kLength2; n++) {
    double s = 0.4 + 0.6*sin((n/10.0 + i/20.0)*2*std::numbers::pi);
    s = std::clamp(s, 0.0, 1.0);
    if (((int) ((n+2)/10.0+i/20.0))%2==0) {
      SetRGB(n+kLength, 0, 50*s, 255*s);
    } else {
      SetRGB(n+kLength, 255*s, 50*s, 0);
    }
  }
  */

  /*
  SetRGB(0+kLength, 0, 50, 255);
  SetRGB(1+kLength, 0, 25, 127);
  SetRGB(46+kLength, 127, 25, 0);
  SetRGB(47+kLength, 255, 50, 0);
  SetRGB(99+kLength, 0, 50, 255);
  SetRGB(98+kLength, 0, 25, 127);
  SetRGB(52+kLength, 127, 25, 0);
  SetRGB(51+kLength, 255, 50, 0);

  int y = 46 - ((i*i)/100)%46;
  SetRGB(y+kLength, 127, 127, 127);
  SetRGB(99-y+kLength, 127, 127, 127);
  /*/
  // LED strip logic
  d1yv+=0.02; d1y+=d1yv;
  if (d1y>46) {
    if (drop_mode) d1y-=46;
    else {
      d1y = 92-d1y;
      d1yv = -d1yv;
      d1s = !d1s;
    }
  }
  if (d1y<0) {
    d1y+=46;
    if (!drop_mode) d1s = !d1s;
  }
  if (d1yv>3) d1yv=3;
  
  d2yv+=0.02; d2y+=d2yv;
  if (d2y>46) {
    if (drop_mode) d2y-=46;
    else {
      d2y = 92-d2y;
      d2yv = -d2yv;
      d2s = !d2s;
    }
  }
  if (d2y<0) {
    d2y+=46;
    if (!drop_mode) d2s = !d2s;
  }
  if (d2yv>3) d2yv=3;

  // LED strip portals
  if (drop_mode) {
    SetRGB(kLength+0,    0, 100, 255);
    SetRGB(kLength+1,    0,  50, 127);
    SetRGB(kLength+46, 127,  50,   0);
    SetRGB(kLength+47, 255, 100,   0);
  } else {
    SetRGB(kLength+0,  255, 100,   0);
    SetRGB(kLength+1,  127,  50,   0);
    SetRGB(kLength+46,   0,  50, 127);
    SetRGB(kLength+47,   0, 100, 255);
  }
  SetRGB(kLength+51, 255, 100,   0);
  SetRGB(kLength+52, 127,  50,   0);
  SetRGB(kLength+97,   0,  50, 127);
  SetRGB(kLength+98,   0, 100, 255);
  SetRGB(kLength+99,   0,   0,   0);
  
  // LED strip dots
  if (d1s) SetRGB(kLength+46-floor(d1y),   0, 100, 255);
  else     SetRGB(kLength+52+floor(d1y),   0, 100, 255);
  
  if (d2s) SetRGB(kLength+46-floor(d2y), 255, 100,   0);
  else     SetRGB(kLength+52+floor(d2y), 255, 100,   0);
  //*/

  Flush();

  i++;
  if (i==200) drop_mode = false;
  if (i==600) {
    drop_mode = true;
    i = 0;
  }
}

void LEDController::SetAngle(double angle) {
  this->angle = angle;
}

int LEDController::pos(int x, int y) {
  if (x<0 || y<0 || x>=16 || y>=16) return -1;
  int nx = 15-x; int ny = 15-y;
  int output = floor(nx/2)*32;
  if (nx%2==0) {
    output+=ny;
  } else {
    output+=31-ny;
  }
  return output;
}

void LEDController::SetRGB(int index, int r, int g, int b) {
  if (index<0 || index>=kTotalLength) return;
  m_ledBuffer[index].SetRGB(r*bright, g*bright, b*bright);
  /*if (index >= kLength) {
    m_ledBuffer[index+kLength2].SetRGB(r*bright, g*bright, b*bright);
  }*/
}

void LEDController::SetRGB(int x, int y, int r, int g, int b) {
  SetRGB(pos(x, y), r, g, b);
}

void LEDController::SetHSV(int index, int h, int s, int v) {
  if (index<0 || index>=kTotalLength) return;
  m_ledBuffer[index].SetHSV(h, s, v*bright);
  /*if (index >= kLength) {
    m_ledBuffer[index+kLength2].SetHSV(h*bright, s*bright, v*bright);
  }*/
}

void LEDController::SetHSV(int x, int y, int h, int s, int v) {
  SetHSV(pos(x, y), h, s, v);
}

void LEDController::Flush() {
  m_led.SetData(m_ledBuffer);
}

void LEDController::SetAlliance(bool isRed) {
  m_allianceIsRed = isRed;
}