// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleBallAutonSwerve extends GoonAutonCommand {
  /** Creates a new SingleBallAutonSwerve. */
  public SingleBallAutonSwerve() {
    // Add your commands in the addCommands() call, e.g.
    
    super.addCommands(
      new InstantCommand(() -> Index.getInstance().setBallCount0()),
      new InstantCommand(() -> Index.getInstance().incrementBallCount()),
      new ParallelRaceGroup(new TakeBallCmd(Index.getInstance()), 
        new SequentialCommandGroup(
          new ToggleIntake(Intake.getInstance()),
          new ToggleShooter(Shooter.getInstance(), 14000),
          new ParallelCommandGroup(
            util.getPathPlannerSwerveControllerCommand(util.singleBallAutonTrajectory()),
            new ToggleHingeCmd(Intake.getInstance())
          ),
          new Pause(1)
          
        )
      ),
      new AimAndShoot(Vision.getInstance(), Shooter.getInstance(), DrivetrainSubsystem.getInstance(), Index.getInstance()),
      new ToggleIntake(Intake.getInstance()),
      new ToggleHingeCmd(Intake.getInstance())
    );

    super.setInitialPose(util.singleBallAutonTrajectory());
  }
}
