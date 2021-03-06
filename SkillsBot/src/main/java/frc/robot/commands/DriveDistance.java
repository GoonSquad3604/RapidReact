// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private DriveTrain m_drive;
  private double m_speed;
  private double m_distance;

  /** Creates a new DriveDistance. */
  public DriveDistance(double speed, double meters, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = speed;
    m_distance = meters;
    m_drive = driveTrain;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.reset();
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    return Math.abs(m_drive.getDistance()) >= m_distance;
  }
}
