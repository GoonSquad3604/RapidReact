// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuton extends SequentialCommandGroup {
  private DriveTrain m_driveTrain;
  private Intake m_intake;
  private Indexer m_indexer;
  private Shooter m_shooter;
  /** Creates a new BasicAuton. */
  public BasicAuton(DriveTrain driveTrain, Intake intake, Indexer indexer, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_driveTrain = driveTrain;
    m_intake = intake;
    m_indexer = indexer;
    m_shooter = shooter;

    //new DriveStraightPID(1, driveTrain)
    addCommands(
    new TurnPID(360.0, driveTrain),
    new TurnPID(-360.0, driveTrain));
  }
}
