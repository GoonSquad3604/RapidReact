// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private CANSparkMax turretMotor = new CANSparkMax(Constants.kTurretId, MotorType.kBrushless);
  private WPI_TalonFX shooterMotor0 = new WPI_TalonFX(Constants.kShooterMotor0Id);
  private WPI_TalonFX shooterMotor1 = new WPI_TalonFX(Constants.kShooterMotor1Id);

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor0.setNeutralMode(NeutralMode.Coast);
    shooterMotor1.setNeutralMode(NeutralMode.Coast);

    shooterMotor0.setInverted(true);
    shooterMotor1.setInverted(true);
    
    turretMotor.getEncoder().setPosition(0);
  }

  public void setShooter(double power) {
    shooterMotor0.set(power);
    shooterMotor1.set(-power);
    
  }

  public void setShooterVelo(double velocity) {
    shooterMotor0.set(ControlMode.Velocity, velocity);
    shooterMotor1.set(ControlMode.Velocity, -velocity);
  }

  public void setTurret(double power) {
    //if ((power > 0 && getTurretPosition() > 15) || (power < 0 && getTurretPosition() < 420  )){ //Funny

      turretMotor.set(power);


   // else {

      //turretMotor.set(0);


   // }
    
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getTurretPosition() {
    return turretMotor.getEncoder().getPosition();
  } //Funny

  public void resetTurret() {

    turretMotor.getEncoder().setPosition(0);

// public static void main i live in great pain

  }
}

















