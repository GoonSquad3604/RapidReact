// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {
  /** Creates a new TwoBallAuton. */
  Drivetrain m_driveTrain; 
  Trajectory m_auton1; 
  Trajectory m_auton2;
  Trajectory m_auton3;
  Intake m_intake;
  Index m_index;
  Shooter m_shooter;

  public TwoBallAuton(Drivetrain driveTrain, Trajectory auton1, Intake intake, Index index, Shooter shooter) {

    m_driveTrain = driveTrain;
    m_auton1 = auton1;
    m_intake = intake;
    m_index = index;
    m_shooter = shooter;

    m_driveTrain.setBrakeMode();

    //m_index.incrementBallCount();

    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

            
    RamseteCommand ramset1 =
    new RamseteCommand(
      m_auton1,
        m_driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_driveTrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveTrain::tankDriveVolts,
        m_driveTrain);

// Reset odometry to the starting pose of the trajectory.
//m_driveTrain.resetOdometry(m_auton1.getInitialPose());
    m_driveTrain.resetOdometry(m_auton1.getInitialPose());


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new ToggleHingeCmd(m_intake), 
      new InstantCommand(() -> m_driveTrain.resetOdometry(m_auton1.getInitialPose())),
      new InstantCommand(() -> m_index.setBallCount0()),
      new InstantCommand(() -> m_index.incrementBallCount()),
      new ParallelRaceGroup(new TakeBallCmd(m_index), 
        new SequentialCommandGroup(
          new ToggleIntake(m_intake),
          new ToggleShooter(m_shooter, 14000),
          new ParallelCommandGroup(
            ramset1, 
            new ToggleHingeCmd(intake)
          ),
          new Pause(1)
          
        )
      ),
      new ParallelCommandGroup
      (
        new InstantCommand(() -> m_index.moveIndex()),
        new Pause(2)
      ),
      new InstantCommand(() -> m_index.stopIndex()),
      //new ShootAll(m_index, m_shooter),
      new InstantCommand(() -> m_driveTrain.setCoastMode()),
      new ToggleIntake(m_intake),
      new ToggleHingeCmd(m_intake)
    );
  }
}
