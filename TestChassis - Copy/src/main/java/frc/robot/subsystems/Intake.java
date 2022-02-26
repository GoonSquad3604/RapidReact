// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import javax.sound.sampled.SourceDataLine;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
 
  private CANSparkMax intakeFront = new CANSparkMax(Constants.kIntakeFront, MotorType.kBrushless);
  private CANSparkMax pivot = new CANSparkMax(Constants.kPivotId, MotorType.kBrushless);
  //private CANSparkMax intakeRear = new CANSparkMax(Constants.kIntakeRear, MotorType.kBrushless);
  
  public Intake() {}

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
}
