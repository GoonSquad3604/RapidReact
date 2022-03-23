// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HingePosition;
import frc.robot.subsystems.Intake;

public class ToggleHingeCmd extends CommandBase {
  private Intake m_intake;
  HingePosition start;
  HingePosition end;
  private boolean moveUp = false;

  /** Creates a new ToggleHingeCmd. */
  public ToggleHingeCmd(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = m_intake.getHingePosition();

    switch(start) {
      case Up:
        end = HingePosition.Down;
        moveUp = false;
        break;
      case Down:
        end = HingePosition.Up;
        moveUp = true;
        break;
      default:
        end = HingePosition.Up;
        moveUp = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(moveUp) {
      m_intake.moveUpAuto();
    }

    else {
      m_intake.moveDownAuto();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intake.getHingePosition() == end)
      return true;
    else return false;
  }
}