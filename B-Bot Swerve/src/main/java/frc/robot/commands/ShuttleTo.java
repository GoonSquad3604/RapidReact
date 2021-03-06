// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ConstantsA;
import frc.robot.subsystems.Shuttle;

public class ShuttleTo extends CommandBase {
  /** Creates a new ShuttleTo. */

  Shuttle m_shuttle;
  double leftPos;
  double rightPos;
  boolean moveForward;
  double max;
  double min;

  public ShuttleTo(double lpos, double rpos, Shuttle shuttle) {
    // Use addRequirements() here to declare subsystem dependencies.

    leftPos = lpos;
    rightPos = rpos;
    m_shuttle = shuttle;
    max = ConstantsA.kShuttleForwardLeft;
    min = ConstantsA.kShuttleBackLeft;
    addRequirements(shuttle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(leftPos > m_shuttle.getEncoderShuttleLeft()) {
      moveForward = true;
    } else moveForward = false;

    //SmartDashboard.putBoolean("move forward", moveForward);
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
//    return (Math.abs(m_shuttle.getEncoderShuttleLeft()-leftPos) < 2) || (m_shuttle.getEncoderShuttleLeft() > max) || (m_shuttle.getEncoderShuttleLeft() < min
 //   || (moveForward && m_shuttle.getLeftForward()) || (!moveForward && m_shuttle.getLeftBack()));

    return (moveForward && m_shuttle.getLeftForward() && m_shuttle.getRightBack()) || (!moveForward && m_shuttle.getLeftBack() && m_shuttle.getRightForward());
  }
}
