// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private WPI_TalonFX shooterMotor0 = new WPI_TalonFX(Constants.kShooterMotor0Id);
  //private WPI_TalonFX shooterMotor1 = new WPI_TalonFX(Constants.kShooterMotor1Id);
  public boolean isRunning = false;
  public double velo;
  
  /** Creates a new Shooter. */
  public Shooter() {
    
    shooterMotor0.setNeutralMode(NeutralMode.Coast);
    //shooterMotor1.setNeutralMode(NeutralMode.Coast);

    shooterMotor0.setInverted(false);
    //shooterMotor1.setInverted(false);

    shooterMotor0.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		shooterMotor0.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    shooterMotor0.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kPIDLoopIdx, Constants.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		shooterMotor0.configNominalOutputForward(0, Constants.kTimeoutMs);
		shooterMotor0.configNominalOutputReverse(0, Constants.kTimeoutMs);
		shooterMotor0.configPeakOutputForward(1, Constants.kTimeoutMs);
		shooterMotor0.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		shooterMotor0.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		shooterMotor0.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		shooterMotor0.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		shooterMotor0.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(isRunning);
    SmartDashboard.putBoolean("Shooter Running", isRunning);
    SmartDashboard.putNumber("shooterVelo", velo);
    SmartDashboard.putNumber("shooter real speed", shooterMotor0.getSelectedSensorVelocity());
  }

  public void setShooter(double power) {
    if(!Constants.isABot) {
      shooterMotor0.set(power);
      //shooterMotor1.set(-power);
    }
    else {
      shooterMotor0.set(power);
    }
    
  }

  public void setShooterVelo(double velocity) {

    //System.out.println("velo: " + velocity);
    shooterMotor0.set(ControlMode.Velocity, velocity);
    velo = velocity;
    //shooterMotor1.set(ControlMode.Velocity, -velocity);
  }

  public double getSpeedForDistance(double meters) {
    double speed = 13500;
    if(meters < 1.9 && meters > 0.5) {
      speed = 6000;
    }
    else if(meters >= 1.9 && meters < 2.25 ) {
      speed = 12000;
    }
    else if(meters >= 2.25 && meters < 2.75 ) {
      speed = 13500;
    }
    else if(meters >= 2.75 && meters < 3.25 ) {
      speed = 14000;
    } 
    else if(meters >= 3.25 && meters < 3.75 ) {
      speed = 14500;
    } 
    else if(meters >= 3.75 && meters < 4.25 ) {
      speed = 15400;
    } 
    else if(meters >= 4.25 && meters < 4.75 ) {
      speed = 16000;
    }  
    else if(meters >= 4.75 && meters < 5.5 ) {
      speed = 17000;
    } 
    else if(meters >= 5.5 && meters < 6.25 ) {
      speed = 17750;
    } 
    else if(meters >= 6.15 ) {
      speed = 18750;
    }  

    return speed + 800;
  }

}
