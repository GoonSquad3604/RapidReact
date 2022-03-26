// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shuttle;

public class ShuttleTo extends CommandBase {
  /** Creates a new ShuttleTo. */

  Shuttle m_shuttle;
  double leftPos;
  double rightPos;
  boolean moveForward;

  public ShuttleTo(double lpos, double rpos, Shuttle shuttle) {
    // Use addRequirements() here to declare subsystem dependencies.

    leftPos = lpos;
    rightPos = rpos;
    m_shuttle = shuttle;
    addRequirements(shuttle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(leftPos > m_shuttle.getEncoderShuttleLeft()) {
      moveForward = true;
    } else moveForward = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(moveForward) m_shuttle.shuttleForward();
    else m_shuttle.shuttleBack();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shuttle.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_shuttle.getEncoderShuttleLeft()-leftPos) < 3);
  }
}
