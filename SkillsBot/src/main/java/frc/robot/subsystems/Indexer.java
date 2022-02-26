// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  CANSparkMax indexMotor0 = new CANSparkMax(Constants.kIndexMotorId, MotorType.kBrushless);
  CANSparkMax indexMotor1 = new CANSparkMax(Constants.kIndexMotor1Id, MotorType.kBrushless);
  DigitalInput sensor0 = new DigitalInput(Constants.kDigitalInputId);



  /** Creates a new Indexer. */
  public Indexer() {
    //indexMotor1.follow(indexMotor0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean detectBall() {
    return !sensor0.get();
  }

  public void runIndexer() {
    indexMotor1.set(.6);
    indexMotor0.set(-.6);
    
  }

  public void stop() {
    indexMotor1.set(0);
    indexMotor0.set(0);
  }
}
