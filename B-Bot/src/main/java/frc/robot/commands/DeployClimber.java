// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shuttle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployClimber extends ParallelCommandGroup {

  private Shuttle m_shuttle;
  private Climber m_climber;

  public DeployClimber(Shuttle shuttle, Climber climber) {

    m_shuttle = shuttle;
    m_climber = climber;

    addCommands(
      new TelescopeTo(Constants.leftTelescopeFull, Constants.rightTelescopeFull, m_climber), 
      new ShuttleTo(Constants.leftShuttleFront-10, Constants.rightShuttleFront+10, m_shuttle)
    );
  }
}
