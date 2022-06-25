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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutonSwerve extends SequentialCommandGroup {
  /** Creates a new TwoBallAutonSwerve. */
  DrivetrainSubsystem m_driveTrain;
  PathPlannerTrajectory path;
  Intake m_intake;
  Vision m_vision;
  Index m_index;
  Shooter m_shooter;

   public TwoBallAutonSwerve(DrivetrainSubsystem driveTrain, PathPlannerTrajectory auton1, Intake intake, Index index, Shooter shooter, Vision vision) {

    PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
      path, 
      m_driveTrain::getPose, 
      m_driveTrain.getKinematics(),
      new PIDController(8, 0, 0), new PIDController(8, 0, 0), 
      new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(m_driveTrain.getAngularVelocity(), m_driveTrain.getAngularVelocity())), 
      m_driveTrain::setStates, 
      m_driveTrain);

    m_driveTrain.resetOdometry(auton1.getInitialPose());

    addCommands(
      //new ToggleHingeCmd(m_intake), 
      
      new InstantCommand(() -> m_index.setBallCount0()),
      new InstantCommand(() -> m_index.incrementBallCount()),
      new ParallelRaceGroup(new TakeBallCmd(m_index), 
        new SequentialCommandGroup(
          new ToggleIntake(m_intake),
          new ToggleShooter(m_shooter, 14000),
          new ParallelCommandGroup(
            swerveCommand,
            new ToggleHingeCmd(intake)
          ),
          new Pause(1)
          
        )
      ),
      new AimAndShoot(m_vision, m_shooter, m_driveTrain, m_index),
      new ToggleIntake(m_intake),
      new ToggleHingeCmd(m_intake)
    );
   }
}
