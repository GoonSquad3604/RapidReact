// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  private int ballCount = 0;

  CANSparkMax indexMotor0 = new CANSparkMax(Constants.kIndexMotorId, MotorType.kBrushed);
  CANSparkMax indexMotor1 = new CANSparkMax(Constants.kIndexMotor1Id, MotorType.kBrushed);
  DigitalInput sensor0 = new DigitalInput(Constants.kSensor0Id);
  DigitalInput sensor1 = new DigitalInput(Constants.kSensor1Id);

  /** Creates a new Index. */
  public Index() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Ball", ballCount > 0);
  }

  public boolean detectBall() {
    return !sensor0.get();
  }

  public boolean detectExit() {
    return !sensor1.get();
  }
  // public boolean noDetect() {
  //   if(ballCount > 0 && sensor1.get()) {
  //     ballCount--;
  //   }
  // }
  public void moveIndexAuto() {

    if(!Constants.isABot) {
      indexMotor0.set(-0.65);
      indexMotor1.set(0.85);
    }
    else {
      indexMotor0.set(-0.65);
      indexMotor1.set(-0.65);   
    }
  }
  public void moveIndex() {

    if(!Constants.isABot) {
      indexMotor0.set(-0.65);
      indexMotor1.set(0.85);
    }
    else {
      indexMotor0.set(-0.85);
      indexMotor1.set(-0.9);   
    }
  }

  public void moveIndex(double pwr) {
    if(!Constants.isABot) {
      indexMotor0.set(- pwr);
      indexMotor1.set(pwr + .2);
    }
    else {
      indexMotor0.set(-pwr);
      indexMotor1.set(-pwr + .2);
    }
  }

  public void reverseIndex() {
    if(!Constants.isABot) {
      indexMotor0.set(0.85);
      indexMotor1.set(-0.85);
    }
    else {
      indexMotor0.set(0.85);
      indexMotor1.set(0.85);   
    }
  }

  public void stopIndex() {
    indexMotor0.set(0);
    indexMotor1.set(0);
 
  }

  public void incrementBallCount() {
    ballCount++;
    if(ballCount > 2) 
      ballCount = 2;
  }

  public void decrementBallCount() {
    ballCount--;
    if(ballCount < 0) {
      ballCount = 0;
    }
  }

  public int getBallCount() {
    return ballCount;
  }
}
