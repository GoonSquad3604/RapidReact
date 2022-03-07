// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax.IdleMode;

public class Drivetrain extends SubsystemBase {

  // Left Motors
  private WPI_TalonFX frontLeftMotor = new WPI_TalonFX(Constants.kFrontLeftId);
  private WPI_TalonFX backLeftMotor = new WPI_TalonFX(Constants.kBackLeftId);

  // Right Motors
  private WPI_TalonFX frontRightMotor = new WPI_TalonFX(Constants.kFrontRightId);
  private WPI_TalonFX backRightMotor = new WPI_TalonFX(Constants.kBackRightId);

  // Follow
  private MotorControllerGroup leftSide = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private MotorControllerGroup rightSide = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private CANCoder canCoderRight = new CANCoder(Constants.kCANCoderRightId);
  private CANCoder canCoderLeft = new CANCoder(Constants.kCANCoderLeftId);
  
  private PigeonIMU pigeon = new PigeonIMU(new WPI_TalonSRX(Constants.kPivotId));

  private DifferentialDrive drive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    rightSide.setInverted(true);
    drive = new DifferentialDrive(leftSide, rightSide);
  }

  public void TankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void ArcadeDrive(double speed, double turn) {
    drive.arcadeDrive(speed, turn);
  }

  public double getDistance() {

    double rightDist = -canCoderRight.getPosition() / Constants.kPulsesPerMeter;
    double leftDist = canCoderLeft.getPosition() / Constants.kPulsesPerMeter;

    return (rightDist+leftDist)/(double)2;
  }

  public double getRightPosition() {
    return canCoderRight.getPosition();
  }

  public double getLeftPosition() {
    return canCoderLeft.getPosition();
  }

  // public void setBrakeMode(){
  //   rightFront.setIdleMode(IdleMode.kBrake);
  //   rightRear.setIdleMode(IdleMode.kBrake);
  //   leftRear.setIdleMode(IdleMode.kBrake);
  //   leftFront.setIdleMode(IdleMode.kBrake);
  // }

  // public void setCoastMode(){
  //   frontRightMotor.setIdleMode(IdleMode.kCoast);
  //   backRightMotor.setIdleMode(IdleMode.kCoast);
  //   backLeftMotor.setIdleMode(IdleMode.kCoast);
  //   leftFront.setIdleMode(IdleMode.kCoast);
  // }


  public double getHeading() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360);
  }

  public void reset() {
    canCoderRight.setPosition(0);
    canCoderLeft.setPosition(0);
    pigeon.setYaw(0);
  } //Funny x2

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}