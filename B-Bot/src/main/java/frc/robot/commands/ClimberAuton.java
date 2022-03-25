// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.tree.TreePath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shuttle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberAuton extends SequentialCommandGroup {

  private Climber m_climber;
  private Shuttle m_shuttle;
  private Drivetrain m_driveTrain; 

  public ClimberAuton(Climber climber, Shuttle shuttle, Drivetrain drivetrain) {
    m_climber = climber;
    m_shuttle = shuttle;
    m_driveTrain = drivetrain;

    addCommands(
      new TelescopeTo(13000, 13000, m_climber),
      new Pause(1),
      new TelescopeTo(90000, -90000, m_climber)
      // new ShuttleTo(Constants.leftShuttleFront - 10, Constants.rightShuttleFront + 10, m_shuttle)
      // new ShuttleTo(Constants.leftShuttleBack, Constants.rightShuttleBack, m_shuttle),
      // new ShuttleTo(0, 0, m_shuttle)
    );
  }
}
