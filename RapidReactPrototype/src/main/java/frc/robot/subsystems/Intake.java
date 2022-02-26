// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private WPI_TalonSRX pivotMotor = new WPI_TalonSRX(Constants.kPivotMotorId);
  private WPI_TalonSRX intakeMotor0 = new WPI_TalonSRX(Constants.kIntakeMotor0Id);
  private WPI_TalonSRX intakeMotor1 = new WPI_TalonSRX(Constants.kIntakeMotor1Id);
  private int hingeTop;
  private int hingeBottom;
  
  private Encoder hingeEncoder = new Encoder(Constants.kHingeEncoderA, Constants.kHingeEncoderB, Constants.kHingeEncoderIndex);

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
