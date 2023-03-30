// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>

#include "Constants.h"

class LEDController : public frc2::SubsystemBase {
 public:
  LEDController();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetState(int state);
  int GetState();
  void SetOverrideState(int state);
  int GetOverrideState();
  int GetPrevState();
  int pos(int x, int y);
  void SetRGB(int index, int r, int g, int b);
  void SetRGB(int x, int y, int r, int g, int b);
  void SetHSV(int index, int h, int s, int v);
  void SetHSV(int x, int y, int h, int s, int v);
  void Flush();
  void Clear();
  void Fill(int r, int g, int b);
  void Pulse(int r, int g, int b, int loop);
  void Circles();
  void Cone();
  void Cube();
  void Cube2();
  void Flash(int i);
  void DrawWord();
  void DrawLetter(char c, int x, int y);
  void DrawAngle();
  void FlashConfirmation();
  void Aperture();
  void SetAngle(double angle); // Takes angle between -18 and 18 (scales up by 10)
  void SetAlliance(bool isRed);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  static constexpr int kLength = 256;  // Length of LED panel
  static constexpr int kLength2 = 100; // Length of LED strips (each)

  static constexpr int kTotalLength = kLength + kLength2; // Total length of all LEDs

  frc::AddressableLED m_led{0};
  std::array<frc::AddressableLED::LEDData, kTotalLength> m_ledBuffer;

  int i = 0;
  int j = 0;
  int y = 0;
  int state = 8;
  int overrideState = -1;
  int prevState = 0;
  int angle = 0;
  bool m_allianceIsRed;

  // Aperture LED strip animation variables
  double d1y = 10;
  double d1yv = 0.4;
  double d1s = false;
  double d2y = 22.2;
  double d2yv = 0.8;
  double d2s = false;
  bool drop_mode = true;

  // LED Brightness from config (shortened to fit more easily in expressions).
  double bright = IOConstants::kLEDBrightness;

  std::string word = "WE HAVE MORE LETTERS NOW";

  std::vector<std::pair<int, int>> aperture_points {
                                {4,0}, {5,0}, {6,0}, {7,0},        {9,0},
                         {3,1}, {4,1}, {5,1}, {6,1}, {7,1}, {8,1},
           {1,2}, {2,2},               {5,2}, {6,2}, {7,2}, {8,2},
           {1,3}, {2,3}, {3,3}, {4,3},               {7,3}, {8,3}, {9,3},
    {0,4}, {1,4}, {2,4}, {3,4}, {4,4}, {5,4},
    {0,5}, {1,5}, {2,5}
  };
};
