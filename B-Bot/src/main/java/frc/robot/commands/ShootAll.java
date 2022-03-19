// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootAll extends CommandBase {
  /** Creates a new ShootAll. */

  Shooter m_shooter;
  Index m_index;
  boolean detected = false;

  public ShootAll(Index index, Shooter shooter) {
    m_index = index;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_shooter.isRunning) {
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_index.detectExit() && !detected) {
      detected = true;
    }

    if(detected) {
      if(!m_index.detectExit() ) {
        m_index.ballCount--;
      }
    }

    m_index.moveIndex();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooter(0);
    m_shooter.isRunning = false;
    m_index.stopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_index.ballCount == 0;
  }
}
