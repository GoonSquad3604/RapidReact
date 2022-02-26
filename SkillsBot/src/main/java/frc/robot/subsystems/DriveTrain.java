// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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

  //Gyro
  private PigeonIMU pigeon = new PigeonIMU(new WPI_TalonSRX(Constants.kPivotMotorId));

  private DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //leftRear.follow(leftFront);
    //rightRear.follow(rightFront, true);

    rightSide.setInverted(true);
    leftSide.setInverted(false);
    //leftFront.setInverted(false);
    drive = new DifferentialDrive(leftSide, rightSide);
    reset();
    setCoastMode();
  }

  public CANSparkMax getLeftFront() {
    return leftFront;
  }

  public void setBrakeMode(){
    rightFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
    leftRear.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    rightFront.setIdleMode(IdleMode.kCoast);
    rightRear.setIdleMode(IdleMode.kCoast);
    leftRear.setIdleMode(IdleMode.kCoast);
    leftFront.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    System.out.println("Angle: " + getHeading());

  }

  public void arcadeDrive(double speed, double turn) {
    drive.arcadeDrive(speed, turn);

    //rightSide.set(.4);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public double getDistance() {

    double rightDist = -rightFront.getEncoder().getPosition() / Constants.kPulsesPerMeter;
    double leftDist = leftFront.getEncoder().getPosition() / Constants.kPulsesPerMeter;

    return (rightDist+leftDist)/(double)2;
  }

  public double getHeading() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return Math.IEEEremainder(ypr[0], 360);
  }


  public void reset() {
    rightFront.getEncoder().setPosition(0);
    leftFront.getEncoder().setPosition(0);
    pigeon.setYaw(0);
  }
}