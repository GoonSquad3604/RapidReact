// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TelescopeTo extends CommandBase {

  Climber m_climber;
  double leftPos;
  double rightPos;
  boolean moveUp;
  boolean moving = false;

  /** Creates a new TelescopeTo. */
  public TelescopeTo(double lpos, double rpos, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    leftPos = lpos;
    rightPos = rpos;
    m_climber = climber;
    addRequirements(climber);
  }

  // public TelescopeTo(Climber m_climber2, int i) {
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(leftPos < m_climber.getTelescopeLeftPosition()) {
      moveUp = false;
    } else moveUp = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(moveUp) m_climber.moveTelescopeUp();
    //else m_climber.moveTelescopeDown();
    if(Math.abs(m_climber.getLeftVelocity()) > 600) {
      moving = true;
    }
    m_climber.setTelescopeToPosition(leftPos);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopTelescopeMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("telescope Calc", (Math.abs(m_climber.getTelescopeLeftPosition()-leftPos)));
    return (moving && Math.abs(m_climber.getLeftVelocity()) < 400);
  }
}
