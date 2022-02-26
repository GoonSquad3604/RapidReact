// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTrain extends SubsystemBase {

  // Declare Motor Objects
 
  private CANSparkMax leftFront = new CANSparkMax(Constants.kLeftFront, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(Constants.kRightFront, MotorType.kBrushless);
  private CANSparkMax leftRear = new CANSparkMax(Constants.kLeftRear, MotorType.kBrushless);
  private CANSparkMax rightRear = new CANSparkMax(Constants.kRightRear, MotorType.kBrushless);

  private MotorControllerGroup leftSide = new MotorControllerGroup(leftFront, leftRear);

  private MotorControllerGroup rightSide = new MotorControllerGroup(rightFront, rightRear);

  private DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //leftRear.follow(leftFront);
    //rightRear.follow(rightFront, true);

    rightSide.setInverted(false);
    leftSide.setInverted(true);
    //leftFront.setInverted(false);
    drive = new DifferentialDrive(leftSide, rightSide);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double speed, double turn) {
    drive.arcadeDrive(speed, -1 * turn);

    //rightSide.set(.4);
  }
}











