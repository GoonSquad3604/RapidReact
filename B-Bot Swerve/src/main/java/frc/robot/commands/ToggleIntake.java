// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {

  Intake m_intake;
  boolean fin = false;

  public ToggleIntake(Intake intake) {
    m_intake = intake;

    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_intake.isToggled) {
      m_intake.take(-1.0);
      m_intake.isToggled = true;
    }

    else {
      m_intake.take(0);
      m_intake.isToggled = false;
    }

    fin = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }
}
