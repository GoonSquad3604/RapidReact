// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.tree.TreePath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberAuton extends SequentialCommandGroup {

  private Climber m_climber;
  
  public ClimberAuton(Climber climber) {
    m_climber = climber;

    addCommands(
    new ClimbTo(m_climber, -100.0),
    new ShuttleTo(m_climber, Constants.kShuttleBack),
    new ClimbTo(m_climber, 100),
    new ClimbTo(m_climber, -100),
    new ShuttleTo(m_climber, Constants.kShuttleFront),
    new ClimbTo(m_climber, 100),
    new ClimbTo(m_climber, -100),
    new ShuttleTo(m_climber, 100));
  }
}
