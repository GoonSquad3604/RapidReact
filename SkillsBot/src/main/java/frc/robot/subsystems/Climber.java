// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    CANSparkMax telescopeMotor0 = new CANSparkMax(Constants.kTelescopeMotor0Id, MotorType.kBrushless);
    CANSparkMax telescopeMotor1 = new CANSparkMax(Constants.kTelescopeMotor1Id, MotorType.kBrushless);
    /** Creates a new ExampleSubsystem. */
    public Climber() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}