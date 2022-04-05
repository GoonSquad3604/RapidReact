// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

  // private CANCoder canCoderRight = new CANCoder(Constants.kCANCoderRightId);
  // private CANCoder canCoderLeft = new CANCoder(Constants.kCANCoderLeftId);
  
  private PigeonIMU pigeon = new PigeonIMU(new WPI_TalonSRX(Constants.kPivotId));

  private DifferentialDrive drive;
  private DifferentialDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
			/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
			configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

      frontRightMotor.configAllSettings(configs);
      frontLeftMotor.configAllSettings(configs);

    reset();
    rightSide.setInverted(true);
    drive = new DifferentialDrive(leftSide, rightSide);
    m_odometry = new DifferentialDriveOdometry(getRotation());
    setCoastMode();
  }

  public void TankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void ArcadeDrive(double speed, double turn) {
    drive.arcadeDrive(speed, turn);
  }

  public double getDistance() {

    double rightDist = getRightPosition() / Constants.kPulsesPerMeter;
    double leftDist = getLeftPosition() / Constants.kPulsesPerMeter;

    return (rightDist+leftDist)/(double)2;
  }

  public double getLeftDistance() {
    return getLeftPosition() / Constants.kPulsesPerMeter;
  }

  public double getRightDistance() {
    return getRightPosition() / Constants.kPulsesPerMeter;
  }

  public double getRightPosition() {
    //return -canCoderRight.getPosition();
    return -1* frontRightMotor.getSelectedSensorPosition(0);
  }

  public double getLeftPosition() {
    return frontLeftMotor.getSelectedSensorPosition(0);
  }

  // public void setBrakeMode(){
  //   rightFront.setIdleMode(IdleMode.kBrake);
  //   rightRear.setIdleMode(IdleMode.kBrake);
  //   leftRear.setIdleMode(IdleMode.kBrake);
  //   leftFront.setIdleMode(IdleMode.kBrake);
  // }

  public void setCoastMode(){
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    backRightMotor.setNeutralMode(NeutralMode.Coast);
    backLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode(){
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double getHeading() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return Math.IEEEremainder( ypr[0], 360);
  }

  public Rotation2d getRotation() {
    //Rotation2d rot = new Rotation2d(getHeading());
    var rot = Rotation2d.fromDegrees(getHeading());
    return rot;
  }

  public void reset() {
    //canCoderRight.setPosition(0);
    //canCoderLeft.setPosition(0);
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
    
    pigeon.setYaw(0);
  } //Funny x2

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftMotor.getSelectedSensorVelocity()* 10 / Constants.kPulsesPerMeter, frontRightMotor.getSelectedSensorVelocity()* -10 / Constants.kPulsesPerMeter);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Encoder", getLeftPosition());;
    // SmartDashboard.putNumber("Right Encoder", getRightPosition());
    // SmartDashboard.putString("Angle2", getRotation().toString());
    //SmartDashboard.putNumber("Right Distance", getRightDistance());
    //SmartDashboard.putNumber("Left Distance", getLeftDistance());

    // SmartDashboard.putString("Post", getPose().toString());
    //SmartDashboard.putString("Velocities", getWheelSpeeds().toString());
    // SmartDashboard.putNumber("rightspeed", frontRightMotor.getSelectedSensorVelocity() * -10 / Constants.kPulsesPerMeter);
    // SmartDashboard.putNumber("leftSpeed",frontLeftMotor.getSelectedSensorVelocity() * 10 / Constants.kPulsesPerMeter);
   
   
    m_odometry.update(
        getRotation(), getLeftDistance(), getRightDistance());

  }

  public void resetOdometry(Pose2d pose) {
    pigeon.setYaw(0);
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
    m_odometry.resetPosition(pose, getRotation());
  }

  public Pose2d getPose() {
    //System.out.println("Pose " + m_odometry.getPoseMeters());
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(rightVolts);
    drive.feed();
  }
}