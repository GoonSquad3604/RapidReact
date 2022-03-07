// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
 
  private CANSparkMax intakeFront = new CANSparkMax(Constants.kIntakeFrontId, MotorType.kBrushless);
  //private CANSparkMax pivot = new CANSparkMax(Constants.kPivotId, MotorType.kBrushless);
  private WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.kPivotId);

  //private CANSparkMax intakeRear = new CANSparkMax(Constants.kIntakeRear, MotorType.kBrushless);

  private int hingeTop;
  private int hingeBottom;
  
  private Encoder hingeEncoder = new Encoder(Constants.kHingeEncoderA, Constants.kHingeEncoderB, Constants.kHingeEncoderIndex);
  
  public Intake() {
    calibrate();
  }

  @Override
  public void periodic() {
    System.out.println("Encoder Value " + hingeEncoder.get());
  }

  public void calibrate() {
    hingeEncoder.reset();
    hingeTop = 0;
    hingeBottom = Constants.kHingeBottomPosition;

    System.out.println("encoder: " + hingeEncoder.get());
    System.out.println("Position: " + getHingePosition().toString());
  }

  public void moveUp() {
    pivot.set(-0.35);
    //System.out.println("move up");
  }

  public void moveDown() {
    pivot.set(0.35);
  }
  
  public void stopPivot() {
    pivot.set(0);
    //System.out.println("stopping");
  }

  public void take(double speed) {
    intakeFront.set(-speed);

    //System.out.println(speed);
    //intakeRear.set(speed);
  }

  public void vomit(double speed) {
    intakeFront.set(speed);
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