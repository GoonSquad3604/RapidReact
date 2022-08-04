// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  private int ballCount = 0;

  CANSparkMax indexMotor0;
  CANSparkMax indexMotor1;
  
  private static Index _instance;
  
   
  DigitalInput sensor0 = new DigitalInput(Constants.kSensor0Id);
  DigitalInput sensor1 = new DigitalInput(Constants.kSensor1Id);

  /** Creates a new Index. */
  public Index() {
    indexMotor0 = new CANSparkMax(Constants.kIndexMotorId, MotorType.kBrushed);
    indexMotor1 = new CANSparkMax(Constants.kIndexMotor1Id, MotorType.kBrushed);

    indexMotor0.setIdleMode(IdleMode.kBrake);
    indexMotor1.setIdleMode(IdleMode.kBrake);
    
  }

  public static final Index getInstance() {
    if (_instance == null) {
            _instance = new Index();
    }
    return _instance;
  } 

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Has Ball", ballCount > 0);
    //SmartDashboard.putNumber("BallCount", ballCount);

    SmartDashboard.putBoolean("Ball0", ballCount>=1);
    SmartDashboard.putBoolean("Ball1", ballCount==2);

  

    SmartDashboard.putBoolean("Sensor 0", detectBall());
    SmartDashboard.putBoolean("Sensor 1", detectExit());
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
      indexMotor0.set(-0.5);
      indexMotor1.set(-0.5);   
    }
  }
  public void moveIndex() {
    
    if(!Constants.isABot) {
      indexMotor0.set(-0.65);
      indexMotor1.set(0.85);
    }
    else {
      indexMotor0.set(-0.9);
      indexMotor1.set(-0.5);   
    }
  }

  public void moveButtomIndex() {
    if(!Constants.isABot) {
      indexMotor1.set(0.85);
      indexMotor0.set(0);
    }
    else {
      indexMotor1.set(-0.5);
      indexMotor0.set(0);
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

  public void setBallCount0() {
    ballCount = 0;
  }
}
