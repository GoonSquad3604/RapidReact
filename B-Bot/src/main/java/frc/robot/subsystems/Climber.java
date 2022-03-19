// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ConstantsA;

public class Climber extends SubsystemBase {
  private CANSparkMax telescopeMotorLeft;
  private CANSparkMax telescopeMotorRight;
  private WPI_TalonFX telescopeMotorLeftA;
  private WPI_TalonFX telescopeMotorRightA;

  private boolean isRunning;

  private final double[] hook1Positions = {178.25450, -178.49255};
  private final double[] hook2Positions = {-157.11439514160156, 161.04244995117188};

  /** Creates a new climbMotors. */
  public Climber() {
    //telescopeMotorLeft.restoreFactoryDefaults();
    //telescopeMotorRight.restoreFactoryDefaults();

    if(!Constants.isABot) {
      telescopeMotorLeft = new CANSparkMax(Constants.kTelescopeMotorLeftId, MotorType.kBrushless);
      telescopeMotorRight = new CANSparkMax(Constants.kTelescopeMotorRightId, MotorType.kBrushless);
    }
    else {
      telescopeMotorLeftA = new WPI_TalonFX(ConstantsA.kTelescopeMotorLeftIdA);
      telescopeMotorLeftA = new WPI_TalonFX(ConstantsA.kTelescopeMotorRightIdA);

    }

    if(Constants.isABot) {
      telescopeMotorRightA.setInverted(true);
      telescopeMotorLeftA.setInverted(true);
    }
    else {
      telescopeMotorRight.setInverted(true);
      telescopeMotorLeft.setInverted(true);
    }


    //telescopeMotorRight.follow(telescopeMotorLeft);
    if(!Constants.isABot) {
      telescopeMotorLeft.setIdleMode(IdleMode.kBrake);
      telescopeMotorRight.setIdleMode(IdleMode.kBrake);
      reset();
    }

    else {
      telescopeMotorLeftA.setNeutralMode(NeutralMode.Brake);
      telescopeMotorRightA.setNeutralMode(NeutralMode.Brake);
      reset();
    }
  }

  //hook1
  //shuttleLeft: 178.25450134277344
  //shuttleRight: -178.4925537109375

//hook 2
  // shuttleLeft: -157.11439514160156
  // shuttleRight: 161.04244995117188

  @Override
  public void periodic() {
     SmartDashboard.putNumber("Telescope Left", getTelescopeLeftPosition());
     SmartDashboard.putNumber("Telescope Right", getTelescopeRightPosition());
    SmartDashboard.putBoolean("Telescopes Running", isRunning);
  }

  public void moveTelescopeUp() {
    
    if(!Constants.isABot) {
      telescopeMotorLeft.set(1.0);
      telescopeMotorRight.set(-1.0);
    }
    else {
      telescopeMotorLeftA.set(1.0);
      telescopeMotorRightA.set(1.0);
    }
    isRunning = true;
  }

  public void moveTelescopeDown() {
    if(!Constants.isABot) {
      telescopeMotorLeft.set(-1.0);
      telescopeMotorRight.set(1.0);
    }
    else {
      telescopeMotorLeftA.set(-1.0);
      telescopeMotorRightA.set(1.0);      
    }
    isRunning = true;
  }

  public void stopTelescopeMotors() {
    if(!Constants.isABot) {
      telescopeMotorLeft.set(0);
      telescopeMotorRight.set(0);
    }
    else {
      telescopeMotorLeftA.set(0);
      telescopeMotorRightA.set(0); 
    }
    isRunning = false;
  }
  // L O L M A O 


  public void getEncoderTelescope() {
    //double
   // if(!Constants.isABot) return telescopeMotorLeft.getEncoder().getPosition();
   // else return telescopeMotorLeftA.
  }

  public void reset() {
    if(!Constants.isABot) {
      telescopeMotorLeft.getEncoder().setPosition(0);
      telescopeMotorRight.getEncoder().setPosition(0);
    }
  }

  public double getTelescopeLeftPosition() {
    return telescopeMotorLeft.getEncoder().getPosition();
  }

  public double getTelescopeRightPosition() {
    return telescopeMotorRight.getEncoder().getPosition();
  }

  public double[] getHook1() {
    return hook1Positions;
  }

  public double[] getHook2() {
    return hook2Positions;
  }
  
}
