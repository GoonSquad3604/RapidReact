// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climbMotors extends SubsystemBase {
  private CANSparkMax motor0 = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax motor1 = new CANSparkMax(5, MotorType.kBrushless);

  /** Creates a new climbMotors. */
  public void moveMotorsCounterClockwise() {
    motor0.set(-0.3);
    motor1.set(0.3);
  }

  public void moveMotorsClockwise() {
    motor0.set(0.3);
    motor1.set(-0.3);
  }


  public void stopMotors() {
    motor0.set(0);
    motor1.set(0);
  }

  public climbMotors() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
