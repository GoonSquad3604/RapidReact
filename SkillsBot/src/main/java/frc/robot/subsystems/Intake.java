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

  public Intake() {
    calibrate();    
  }

  @Override
  public void periodic() {
  }

  public void calibrate() {
    hingeEncoder.reset();
    hingeTop = 0;
    hingeBottom = 350;

    System.out.println("encoder: " + hingeEncoder.get());
    System.out.println("Position: " + getHingePosition().toString());
  }

  public void take() {
    intakeMotor0.set(-0.8);
    intakeMotor1.set(0.8);
  }

  public void stop() {
    intakeMotor0.set(0);
    intakeMotor1.set(0);
  }

  public void moveUp() {
    pivotMotor.set(0.5);
  }

  public void moveDown() {
    pivotMotor.set(-0.5);
  }

  public void stopPivot() {
    pivotMotor.set(0);
  }

  public HingePosition getHingePosition() {
    if(hingeEncoder.get() < hingeTop+20 ) {
      return HingePosition.Up;
    }

    else if(hingeEncoder.get() > hingeBottom-40) {
      return HingePosition.Down;
    }

    return HingePosition.Between;

  }

  

}