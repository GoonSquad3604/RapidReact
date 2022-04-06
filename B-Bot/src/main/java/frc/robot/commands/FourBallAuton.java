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
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuton extends SequentialCommandGroup {
  Drivetrain m_driveTrain; 
  Trajectory m_auton1; 
  Trajectory m_auton2;
  Trajectory m_auton3;
  Intake m_intake;
  Index m_index;
  Shooter m_shooter;
  Vision m_vision;

  /** Creates a new FourBallAuton. */
  public FourBallAuton(Drivetrain driveTrain, Trajectory auton1, Trajectory auton2, Trajectory auton3, Intake intake, Index index, Shooter shooter, Vision vision) {
    
    m_driveTrain = driveTrain;
    m_auton1 = auton1;
    m_auton2 = auton2;
    m_auton3 = auton3;
    m_intake = intake;
    m_index = index;
    m_shooter = shooter;
    m_vision = vision;

    //m_driveTrain.setBrakeMode();
            
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

    RamseteCommand ramset2 =
    new RamseteCommand(
      m_auton2,
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

    RamseteCommand ramset3 =
    new RamseteCommand(
      m_auton3,
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

    addCommands(

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
      // new ParallelCommandGroup
      // (
      //   new InstantCommand(() -> m_index.moveIndex()),
      //   new Pause(2)
      // ),
      new AimAndShoot(m_vision, m_shooter, m_driveTrain, m_index),
      new ParallelRaceGroup(
        new TakeBallCmd(m_index),
        new SequentialCommandGroup(
          ramset2,
          new Pause(2)
        )
      ),
      new ToggleShooter(m_shooter, m_vision, true),
      ramset3,
      new AimAndShoot(m_vision, m_shooter, m_driveTrain, m_index),
      new InstantCommand(() -> m_driveTrain.setCoastMode()),
      new ToggleIntake(m_intake),
      new ToggleHingeCmd(m_intake)
    );
  }
}
