// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

  // Subsystem methods go here.
  void SetState(int state);
  int GetState();
  int pos(int x, int y);
  void SetRGB(int index, int r, int g, int b);
  void SetRGB(int x, int y, int r, int g, int b);
  void SetHSV(int index, int h, int s, int v);
  void SetHSV(int x, int y, int h, int s, int v);
  void Flush();
  void Clear();
  void Circles();
  void Cone();
  void Cube();
  void Cube2();
  void Flash(int i);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  static constexpr int kLength = 512;

  frc::AddressableLED m_led{0};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;  // Reuse the buffer

  int i = 0;
  int y = 0;
  int state = 0;
};
