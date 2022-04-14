// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootAllDumb extends CommandBase {
  /** Creates a new ShootAllDumb. */
  Shooter m_shooter;
  Index m_index;
  Vision m_vision;
  boolean detected = false;

  public ShootAllDumb(Index index, Vision vision, Shooter shooter) {
    m_index = index;
    m_shooter = shooter;
    m_vision = vision;
    detected = false;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_index.detectExit() && !detected) {
      detected = true;
    }

    if(detected){
      if(!m_index.detectExit()) {
        m_index.decrementBallCount();
        detected = false;
      }
      if(m_index.getBallCount() == 1) m_shooter.setShooterVelo(m_shooter.getSpeedForDistance(m_vision.getDistance())+100);
    }

    m_index.moveIndex();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.stopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_index.getBallCount() == 0;
  }
}
