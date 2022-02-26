// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climbMotors extends SubsystemBase {
  private CANSparkMax motor0 = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax motor1 = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax telescopeMotor0 = new CANSparkMax(8, MotorType.kBrushless);
  private final double[] hook1Positions = {178.25450, -178.49255};
  private final double[] hook2Positions = {-157.11439514160156, 161.04244995117188};

  /** Creates a new climbMotors. */
  public climbMotors() {
    reset();
  }

  //hook1
  //Motor0: 178.25450134277344
  //Motor1: -178.4925537109375

//hook 2
  // Motor0: -157.11439514160156
  // Motor1: 161.04244995117188

  @Override
  public void periodic() {
    System.out.print("Motor0: " + getEncoderMotor0() +
    "\n Motor1: " + getEncoderMotor1() +
    "\n Telescope: " + getEncoderTelescope() + "\n");

    SmartDashboard.putNumber("Motor0", getEncoderMotor0());
    SmartDashboard.putNumber("Motor1", getEncoderMotor1());
    SmartDashboard.putNumber("Telescopic", getEncoderTelescope());
  }

  public void moveMotorsCounterClockwise() {
    motor0.set(-0.3);
    motor1.set(0.3);
  }

  public void moveMotorsClockwise() {
    motor0.set(0.3);
    motor1.set(-0.3);
  }

  public void moveTelescopeUp() {
    telescopeMotor0.set(0.4);
  }

  public void moveTelescopeDown() {
    telescopeMotor0.set(-0.4);
  }

  public void stopMotors() {
    motor0.set(0);
    motor1.set(0);
  }

  public void stopTelescopeMotors() {
    telescopeMotor0.set(0);
  }

  public double getEncoderTelescope() {
    return telescopeMotor0.getEncoder().getPosition();
  }

  public double getEncoderMotor0() {
    return motor0.getEncoder().getPosition();
  }

  public double getEncoderMotor1() {
    return motor1.getEncoder().getPosition();
  }

  public void reset() {
    motor1.getEncoder().setPosition(0); // lol 69
    motor0.getEncoder().setPosition(0);
    telescopeMotor0.getEncoder().setPosition(0);
  }

  public double[] getHook1() {
    return hook1Positions;
  }

  public double[] getHook2() {
    return hook2Positions;
  }
  
}
