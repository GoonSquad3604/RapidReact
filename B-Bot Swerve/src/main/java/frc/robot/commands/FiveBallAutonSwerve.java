// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
public class FiveBallAutonSwerve extends GoonAutonCommand {
  /** Creates a new FiveBallAutonSwerve. */
  public FiveBallAutonSwerve() {
    
    super.addCommands(
      new ToggleHingeCmd(Intake.getInstance()), 
      
      new InstantCommand(() -> Index.getInstance().setBallCount0()),
      new InstantCommand(() -> Index.getInstance().incrementBallCount()),
      new ParallelRaceGroup(new TakeBallCmd(Index.getInstance()), 
        new SequentialCommandGroup(
          new ToggleIntake(Intake.getInstance()),
          new ToggleShooter(Shooter.getInstance(), 14000),
          new ParallelCommandGroup(
            util.getPathPlannerSwerveControllerCommand(util.fiveBall1Trajectory()),
            new ToggleHingeCmd(Intake.getInstance())
          ),
          new Pause(1)
          
        )
      ),
      new ToggleIntake(Intake.getInstance()),
      new AutonShoot(),
      
      //second path to shoot middle ball
      new ParallelRaceGroup(new TakeBallCmd(Index.getInstance()), 
        new SequentialCommandGroup(
          new ToggleIntake(Intake.getInstance()),
          new ToggleShooter(Shooter.getInstance(), 14000, true),
          util.getPathPlannerSwerveControllerCommand(util.fiveBall2Trajectory()),
          new Pause(1)
        )
      ),
      new ToggleIntake(Intake.getInstance()),
      new AutonShoot(),
      // Third path to grab balls from human player station
      new ParallelRaceGroup(new TakeBallCmd(Index.getInstance()), 
        new SequentialCommandGroup(
          new ToggleIntake(Intake.getInstance()),
          new ToggleShooter(Shooter.getInstance(), 14000, true),
          util.getPathPlannerSwerveControllerCommand(util.fiveBall3Trajectory()),
          new Pause(1),
          // Fourth path to go shoot
          util.getPathPlannerSwerveControllerCommand(util.fiveBall4Trajectory())
        )
      ),
      new ToggleIntake(Intake.getInstance()),
      new AutonShoot(),
      new ToggleHingeCmd(Intake.getInstance())
    );

    super.setInitialPose(util.fiveBall1Trajectory());
  }
}
