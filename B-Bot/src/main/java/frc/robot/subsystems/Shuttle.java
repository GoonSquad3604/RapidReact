// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shuttle extends SubsystemBase {

  private CANSparkMax shuttleLeft = new CANSparkMax(Constants.kShuttleMotorLeftId, MotorType.kBrushless);
  private CANSparkMax shuttleRight = new CANSparkMax(Constants.kShuttleMotorRightId, MotorType.kBrushless);
  private SparkMaxLimitSwitch leftForward;
  private SparkMaxLimitSwitch rightForward;
  private SparkMaxLimitSwitch rightBack;
  private SparkMaxLimitSwitch leftBack;

  /** Creates a new Shuttle. */
  public Shuttle() {
    shuttleLeft.setIdleMode(IdleMode.kBrake);
    shuttleRight.setIdleMode(IdleMode.kBrake);
    reset();
    leftForward = shuttleLeft.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rightForward = shuttleRight.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    leftBack = shuttleLeft.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rightBack = shuttleRight.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    leftForward.enableLimitSwitch(true);
    rightForward.enableLimitSwitch(true);
    leftBack.enableLimitSwitch(true);
    rightBack.enableLimitSwitch(true);

  }

  public void shuttleBack() {
    shuttleLeft.set(-0.9);
    shuttleRight.set(0.9);
  }

  public void shuttleForward() {
    shuttleLeft.set(0.9);
    shuttleRight.set(-0.9);
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
  }
  
  public void stopMotors() {
    shuttleLeft.set(0);
    shuttleRight.set(0);
  }

  // Return limit switches
  public boolean getLeftForward() {
    return leftForward.isPressed();
  }
  public boolean getRightForward() {
    return rightForward.isPressed();
  }
  public boolean getLeftBack() {
    return leftBack.isPressed();
  }
  public boolean getRightBack() {
    return rightBack.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shuttle Left", getEncoderShuttleLeft());
    SmartDashboard.putNumber("Shuttle Right", getEncoderShuttleRight());

    SmartDashboard.putBoolean("Press Left-Back", leftBack.isPressed());
    SmartDashboard.putBoolean("Press Right-Back", rightBack.isPressed());
    SmartDashboard.putBoolean("Press Left-Forward", leftForward.isPressed());
    SmartDashboard.putBoolean("Press Right-Forward", rightForward.isPressed());

    
  }
}
