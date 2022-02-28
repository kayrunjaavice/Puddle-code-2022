// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <math.h>


//frc::Compressor pcmCompressor{11, frc::PneumaticsModuleType::CTREPCM};

double currentTimeStamp = 0, lastTimestamp = 0, dt = 0;
double matchTime = 0;

int feedState = 0;

void Robot::RobotInit() {

  //frc::Compressor disable();
  
  driverXbox = new frc::XboxController(0);
  operatorXbox = new frc::XboxController(1);

  flDrive = new TalonSRX(0);
  frDrive = new TalonSRX(1);
  blDrive = new TalonSRX(2);
  brDrive = new TalonSRX(3);

  leftIntake = new TalonSRX(4);
  rightIntake = new TalonSRX(5);

  indexOne = new TalonSRX(6);
  indexTwo = new TalonSRX(7);

  lowerShooter = new TalonSRX(8);
  upperShooter = new VictorSPX(9);

  anglePiston = new frc::DoubleSolenoid(11, frc::PneumaticsModuleType::CTREPCM, 0, 1);
  anglePiston->Set(frc::DoubleSolenoid::kForward);

  climber = new frc::DoubleSolenoid(11, frc::PneumaticsModuleType::CTREPCM, 2, 3);
  climber->Set(frc::DoubleSolenoid::kForward);

  auto camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera.SetFPS(30);
  camera.SetResolution(200, 200);

  feedTime = new frc::Timer;
}

void Robot::RobotPeriodic() {
  // DT conversion
  matchTime = (double)frc::Timer::GetMatchTime();
  currentTimeStamp = (double)frc::Timer::GetFPGATimestamp();
  dt = currentTimeStamp - lastTimestamp;

  lastTimestamp = currentTimeStamp;
}
double timeAtTheStart = 0;

void Robot::AutonomousInit() {
  
  timeAtTheStart = currentTimeStamp;
  anglePiston->Set(frc::DoubleSolenoid::kForward);


}
void Robot::AutonomousPeriodic() {
 

  //-------------------Autonomous-------------------

  if(currentTimeStamp-timeAtTheStart >= 0 && currentTimeStamp-timeAtTheStart < 1.5){

      flDrive->Set(ControlMode::PercentOutput, currentTimeStamp-timeAtTheStart);
      blDrive->Set(ControlMode::PercentOutput, -(currentTimeStamp-timeAtTheStart));
      frDrive->Set(ControlMode::PercentOutput, -(currentTimeStamp-timeAtTheStart));
      brDrive->Set(ControlMode::PercentOutput, -(currentTimeStamp-timeAtTheStart));


    lowerShooter->Set(ControlMode::PercentOutput, 1);
    upperShooter->Set(ControlMode::PercentOutput, 1);
  }

  if(currentTimeStamp-timeAtTheStart >= 1.5 && currentTimeStamp-timeAtTheStart < 4){
    flDrive->Set(ControlMode::PercentOutput, -0);
    blDrive->Set(ControlMode::PercentOutput, 0);
    frDrive->Set(ControlMode::PercentOutput, 0);
    brDrive->Set(ControlMode::PercentOutput, 0);
    indexOne->Set(ControlMode::PercentOutput, 0.7);
    indexTwo->Set(ControlMode::PercentOutput, 0.7);
  }

  if(currentTimeStamp-timeAtTheStart >= 4 && currentTimeStamp-timeAtTheStart < 7){
  indexOne->Set(ControlMode::PercentOutput, 0);
  indexTwo->Set(ControlMode::PercentOutput, 0);
  lowerShooter->Set(ControlMode::PercentOutput, 0);
  upperShooter->Set(ControlMode::PercentOutput, 0);
  }

  std::cout<<currentTimeStamp-timeAtTheStart<<std::endl;


}

void Robot::TeleopInit() {}
  bool climberz = false;
  int cState = 0;
void Robot::TeleopPeriodic() {



  //-------------------Drive Code-------------------

  //Get values
  double transX = (-driverXbox->GetLeftY())/*(-driverXbox->GetLeftY())*(-driverXbox->GetLeftY())*/;
  double transY = driverXbox->GetLeftX()/*(driverXbox->GetLeftX())*(driverXbox->GetLeftX())*/;

  double rotate = driverXbox->GetRightX()/*driverXbox->GetRightX()*driverXbox->GetRightX()*/;

  //wheel speed math
  double den = std::max(abs(transX)+abs(transY)+abs(rotate), 1.0);

  double frontL = (transY - transX - rotate)/den;
  double backR = (transY + transX + rotate)/den;

  double backL = (-transY + transX - rotate)/den;
  double frontR = (transY + transX - rotate)/den;

  //assign wheel speed
  flDrive->Set(ControlMode::PercentOutput, frontL);
  frDrive->Set(ControlMode::PercentOutput, frontR);
  blDrive->Set(ControlMode::PercentOutput, backL);
  brDrive->Set(ControlMode::PercentOutput, backR);



  //-------------------Indexing + Feeding + Shooting-------------------

  //get control values
  bool lBumper = driverXbox->GetLeftBumper();
  bool rBumper = driverXbox->GetRightBumper();

  bool intake = driverXbox->GetYButton();
  //bool reverseIntake = driverXbox ->GetYButton();

  bool reverseFeed = driverXbox->GetXButton();

  double feedTrigger = driverXbox->GetLeftTriggerAxis();
  double shootTrigger = driverXbox->GetRightTriggerAxis();

  bool topIntake = driverXbox->GetBButton();
  climberz = driverXbox->GetAButton();

  //climber state
  if(climberz == true){
    climber->Set(frc::DoubleSolenoid::kForward);
  }
  else{
    climber->Set(frc::DoubleSolenoid::kReverse);
  }

  //shooter angle state
  if(lBumper == true && rBumper == false){
    anglePiston->Set(frc::DoubleSolenoid::kForward);
  }
  if(rBumper == true && lBumper == false){
    anglePiston->Set(frc::DoubleSolenoid::kReverse);
  }



  //-------------------Intake-------------------

  if(intake == true) {
    leftIntake->Set(ControlMode::PercentOutput, -1);
    rightIntake->Set(ControlMode::PercentOutput, -1);
  } else {
    //if(reverseIntake == true){
    //  leftIntake->Set(ControlMode::PercentOutput, -0.7);
    //  rightIntake->Set(ControlMode::PercentOutput, -0.7);
    //} else {
      leftIntake->Set(ControlMode::PercentOutput, 0);
      rightIntake->Set(ControlMode::PercentOutput, 0);
    //}
  }

  //feed motors
  if(feedTrigger >=0.2 && feedState == 0){

    feedTime->Start();

    indexOne->Set(ControlMode::PercentOutput, 0.7);
    indexTwo->Set(ControlMode::PercentOutput, 0.7);

    if((double)feedTime->Get() >= 0.2){
      
      feedState = 1;

      indexOne->Set(ControlMode::PercentOutput, 0);
      indexTwo->Set(ControlMode::PercentOutput, 0);
      }
    }
  if(feedTrigger <= 0.2){
    if(reverseFeed == true){
      indexOne->Set(ControlMode::PercentOutput, -0.6);
      indexTwo->Set(ControlMode::PercentOutput, -0.6);
    }else{
      indexOne->Set(ControlMode::PercentOutput, 0);
      indexTwo->Set(ControlMode::PercentOutput, 0);

      feedState = 0;
      feedTime->Stop();
      feedTime->Reset();
    }
  }

  //shoot motors
  if(shootTrigger >= 0.1 && shootTrigger < 0.6){
    lowerShooter->Set(ControlMode::PercentOutput, -0.42);
    upperShooter->Set(ControlMode::PercentOutput, -0.42);
  }
  if(shootTrigger >= 0.6){
    lowerShooter->Set(ControlMode::PercentOutput, -1);
    upperShooter->Set(ControlMode::PercentOutput, -1);
  }
  if(shootTrigger < 0.1){
    if(topIntake == true){
      lowerShooter->Set(ControlMode::PercentOutput, 0.5);
      upperShooter->Set(ControlMode::PercentOutput, 0.5);
    }else{
      lowerShooter->Set(ControlMode::PercentOutput, 0);
      upperShooter->Set(ControlMode::PercentOutput, 0);
    }
  }

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
