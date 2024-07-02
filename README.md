# Winner's Circle Robotics 2023 Rep

Team 5492's Swerve Robot code for the 2023 game [Charged Up!](https://firstroboticsbc.org/first-robotics-competition/charged-up-game-and-season/) (Based on MAXSwerve's 2023 [C++ Template](https://github.com/REVrobotics/MAXSwerve-Cpp-Template))

## Authors

- Robin 
- Zach

## This Year's Goals

- Teleop Automation
- Swerve

## Game Info

[Charged Up!](https://firstroboticsbc.org/first-robotics-competition/charged-up-game-and-season/)

## Tools

### Standard Tools

- [WPILib 2023](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
    - [What's new in 2023](https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html)
- [Limelight](https://docs.limelightvision.io/en/latest/)

### VsCode Extensions

- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools): IntelliSense & Beautifier
    - Uses [Clang syntax](https://clang.llvm.org/docs/ClangFormatStyleOptions.html) for settings
    - [Clang Format Quick Tutorial](https://leimao.github.io/blog/Clang-Format-Quick-Tutorial/)

### FRC Libraries
- NavX
- REVLib
- FRC New Command Lib

### Outside Libraries

- [Nlohmann Json](https://github.com/nlohmann/json)

## Helpful Links

- [Google's c++ Style Guide](https://google.github.io/styleguide/cppguide.html)

## Tasks for the 2023 Season

- [x] Update code for new swerve modules
- [x] Add variable speed
- [x] Fix acceleration?
- [ ] Write autonomous routines
    - [ ] Routine A: Score two from furthest-from-loading-bay
    - [x] Routine B: Score one
    - [x] Routine C: Score one and auto-balance
    - [x] Routine D: Taxi
    - [ ] Finalize list
- [x] Create subsystems for robot code
    - [x] Drivebase
        - [ ] Snap to rotation
    - [x] Arm
    - [x] Collector
- [x] Controller Design
    - [x] Document and share controller design
- [ ] Tele-auto scoring
    - [x] Detect AprilTags
    - [x] Align robot to AprilTags
    - [x] Align robot to reflective tape
    - [x] Auto extend arm to right height
    - [ ] Auto score
- [x] Tele-auto balance
    - [x] Get balance of robot
    - [x] Maneuver robot based on balance
- [ ] Tele-auto loading
    - [x] AprilTags work for scoring repurposed
    - [ ] Auto collect
- [x] Tele-auto arm levels
- [ ] Color detection?
- [x] Driver camera
- [x] Limelight 3 configured
- [ ] ~~Auto-mono-piece-control~~
- [x] LED Panel
    - [x] Request indicator for human player
    - [ ] ~~Align indicator based on reflective tape in case Limelight visual fails~~
- [ ] Sound effects!
- [x] Care for code
    - [x] Document
    - [x] Refactor
    - [x] Level based logging
        - [x] Update to include units library
    - [x] JSON/YAML config instead of flat text

## Reference

https://github.com/REVrobotics/MAXSwerve-Cpp-Template/
