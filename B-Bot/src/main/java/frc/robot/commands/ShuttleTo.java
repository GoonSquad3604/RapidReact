// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ShuttleTo extends CommandBase {
  /** Creates a new ShuttleForward. */

  public Climber m_climber;
  private double toPosition;
  private int direction;

  public ShuttleTo(Climber climber, double position) {
    m_climber = climber;
    toPosition = position;

    if(m_climber.getEncoderShuttleLeft() > toPosition) direction = -1;
    else direction = 1; 

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_climber.getEncoderShuttleLeft() > toPosition)
      m_climber.moveMotorsClockwise();
    else 
      m_climber.moveMotorsCounterClockwise();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(direction == 1) return m_climber.getEncoderShuttleLeft() < toPosition;
    else return m_climber.getEncoderShuttleLeft() > toPosition;
  }
}
