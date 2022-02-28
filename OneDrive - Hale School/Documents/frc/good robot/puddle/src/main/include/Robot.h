// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <cameraserver/CameraServer.h>
#include <frc/Timer.h>
#include <iostream>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  void setDrivetrain(double leftPower, double rightPower);

  frc::XboxController *driverXbox, *operatorXbox;
  TalonSRX *flDrive, *blDrive, *frDrive, *brDrive, *leftIntake, *rightIntake, *indexOne, *indexTwo, *lowerShooter;
  VictorSPX *upperShooter;
  frc::DoubleSolenoid *anglePiston, *climber;
  frc::Timer *feedTime;
  double feedPulse = 0;
};
