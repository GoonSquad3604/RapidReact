// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax shuttleLeft = new CANSparkMax(Constants.kShuttleMotorLeftId, MotorType.kBrushless);
  private CANSparkMax shuttleRight = new CANSparkMax(Constants.kShuttleMotorRightId, MotorType.kBrushless);
  private CANSparkMax telescopeMotorLeft = new CANSparkMax(Constants.kTelescopeMotorLeftId, MotorType.kBrushless);
  private CANSparkMax telescopeMotorRight = new CANSparkMax(Constants.kTelescopeMotorRightId, MotorType.kBrushless);
  private final double[] hook1Positions = {178.25450, -178.49255};
  private final double[] hook2Positions = {-157.11439514160156, 161.04244995117188};

  /** Creates a new climbMotors. */
  public Climber() {
    //telescopeMotorLeft.restoreFactoryDefaults();
    //telescopeMotorRight.restoreFactoryDefaults();


    telescopeMotorRight.setInverted(true);
    telescopeMotorLeft.setInverted(true);
    //telescopeMotorRight.follow(telescopeMotorLeft);
    
    telescopeMotorLeft.setIdleMode(IdleMode.kBrake);
    telescopeMotorRight.setIdleMode(IdleMode.kBrake);
    reset();
  }

  //hook1
  //shuttleLeft: 178.25450134277344
  //shuttleRight: -178.4925537109375

//hook 2
  // shuttleLeft: -157.11439514160156
  // shuttleRight: 161.04244995117188

  @Override
  public void periodic() {
    // System.out.println("Left Shuttle: " + getEncoderShuttleLeft());
    // System.out.println("Right Shuttle: " + getEncoderShuttleRight());
    // System.out.println("Left Telescope" + getTelescopeLeftPosition());
    // System.out.println("Right Telescope" + getTelescopeRightPosition());
  }

  public void moveMotorsCounterClockwise() {
    shuttleLeft.set(-0.3);
    shuttleRight.set(0.3);
  }

  public void moveMotorsClockwise() {
    shuttleLeft.set(0.3);
    shuttleRight.set(-0.3);
  }

  public void moveTelescopeUp() {
    
    telescopeMotorLeft.set(0.4);
    telescopeMotorRight.set(-0.4);
  }

  public void moveTelescopeDown() {
    telescopeMotorLeft.set(-0.4);
    telescopeMotorRight.set(0.4);
  }

  public void stopMotors() {
    shuttleLeft.set(0);
    shuttleRight.set(0);
  }

  public void stopTelescopeMotors() {
    telescopeMotorLeft.set(0);
    telescopeMotorRight.set(0);
  }
  // L O L M A O 


  public double getEncoderTelescope() {
    return telescopeMotorLeft.getEncoder().getPosition();
  }

  public double getEncoderShuttleLeft() {
    return shuttleLeft.getEncoder().getPosition();
  }

  public double getEncoderShuttleRight() {
    return shuttleRight.getEncoder().getPosition();
  }

  public void reset() {
    shuttleRight.getEncoder().setPosition(0);
    shuttleLeft.getEncoder().setPosition(0);
    //telescopeshuttleLeft.getEncoder().setPosition(0);
  }

  public double getTelescopeLeftPosition() {
    return telescopeMotorLeft.getEncoder().getPosition();
  }

  public double getTelescopeRightPosition() {
    return telescopeMotorRight.getEncoder().getPosition();
  }

  public double[] getHook1() {
    return hook1Positions;
  }

  public double[] getHook2() {
    return hook2Positions;
  }
  
}
