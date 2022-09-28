// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ConstantsA;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
 
  private CANSparkMax intakeFront = new CANSparkMax(Constants.kIntakeFrontId, MotorType.kBrushed);
  //private CANSparkMax pivot = new CANSparkMax(Constants.kPivotId, MotorType.kBrushless);
  private WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.kPivotId);
  //private CANSparkMax intakeRear = new CANSparkMax(Constants.kIntakeRear, MotorType.kBrushless);

  private int hingeTop;
  private int hingeBottom;
  public boolean isToggled = false;
  private static Intake _instance;
  
  private Encoder hingeEncoder = new Encoder(Constants.kHingeEncoderA, Constants.kHingeEncoderB, Constants.kHingeEncoderIndex);
  
  public Intake() {
    calibrate();
  }

  public static final Intake getInstance() {
    if (_instance == null) {
            _instance = new Intake();
    }
    return _instance;
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current", getIntakeCurrent());
    
  }

  public void calibrate() {
    hingeEncoder.reset();
    hingeTop = 0;
    hingeBottom = Constants.isABot ? ConstantsA.kHingeBottomPositionA : Constants.kHingeBottomPosition;

    System.out.println("encoder: " + hingeEncoder.get());
    System.out.println("Position: " + getHingePosition().toString());
  }

  public void moveUp() {
    pivot.set(-0.85);
    //System.out.println("move up");
  }

  public void moveDown() {
    pivot.set(0.85);
  }

  public void moveUpAuto() {
    pivot.set(-0.85);
    //System.out.println("move up");
  }

  public void moveDownAuto() {
    pivot.set(0.85);
  }
  
  public void stopPivot() {
    pivot.set(0);
    //System.out.println("stopping");
  }

  public void take(double speed) {
    
    if(getIntakeCurrent() > .5) {
      intakeFront.set(0);
    }
    else {
      intakeFront.set(-speed);
    }
  }

  public void vomit(double speed) {
    intakeFront.set(speed);
  }

  public double getIntakeCurrent() {

    double current = intakeFront.getOutputCurrent();

    return current;
  }

  public HingePosition getHingePosition() {
    if(hingeEncoder.get() > hingeTop-20 ) {
      return HingePosition.Up;
    }

    else if(hingeEncoder.get() < hingeBottom+40) {
      return HingePosition.Down;
    }

    return HingePosition.Between;

  }
}