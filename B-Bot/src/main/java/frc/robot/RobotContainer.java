// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.commands.ClimberAuton;
import frc.robot.commands.SetShooterToPowerCmd;
import frc.robot.commands.SetShooterToSpeedCmd;
import frc.robot.commands.TakeBallCmd;
import frc.robot.commands.ToggleHingeCmd;
import frc.robot.commands.ToggleShooter;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  
  private final Drivetrain m_driveTrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Index m_indexer = new Index();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Vision m_Vision = new Vision();
  XboxController m_driverController = new XboxController(0);
  XboxController m_operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putNumber("shooter speed", 4000);
    SmartDashboard.putNumber("shooterPower", .5);

    m_driveTrain.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new RunCommand(
          () ->
              m_driveTrain.ArcadeDrive(
                  -m_driverController.getLeftY(), m_driverController.getRightX()),
          m_driveTrain)
      );

    // Configure the button bindings
    configureButtonBindings();
  }
  // Lol 69

  private void configureButtonBindings() {

    // Operator joysticks
    final JoystickButton operatorAButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
    final JoystickButton operatorYButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
    final JoystickButton operatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
    final JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
    final JoystickButton operatorXButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
    final JoystickButton operatorStartButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);
    final JoystickButton operatorBButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
    final JoystickButton operatorBackButton = new JoystickButton(m_operatorController, XboxController.Button.kBack.value);

    // Operator Triggers
    final Trigger operatorRightTriggerP = new Trigger(() -> m_operatorController.getRightTriggerAxis() >= .5);
    final Trigger operatorLeftTriggerP = new Trigger(() -> m_operatorController.getLeftTriggerAxis() >= .5);
    final Trigger operatorRightTriggerR = new Trigger(() -> m_operatorController.getRightTriggerAxis() < .5);
    final Trigger operatorLeftTriggerR = new Trigger(() -> m_operatorController.getLeftTriggerAxis() < .5);

    // Driver joysticks
    final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    final JoystickButton driverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    final JoystickButton driverLeftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

    //--------------------------------------------------
    // Operator Controls!
    //--------------------------------------------------
  
    // Telescope motors
    operatorBackButton.whenHeld(new RunCommand(() -> m_climber.moveTelescopeUp(), m_climber));
    operatorStartButton.whenHeld(new RunCommand(() -> m_climber.moveTelescopeDown(), m_climber));
    operatorBackButton.whenInactive(new InstantCommand(() -> m_climber.stopTelescopeMotors(), m_climber));
    operatorStartButton.whenInactive(new InstantCommand(() -> m_climber.stopTelescopeMotors(), m_climber));
  
    // Move hinge
    operatorRightTriggerP.whenActive(new InstantCommand(() -> m_intake.moveDown()));
    operatorLeftTriggerP.whenActive(new InstantCommand(() -> m_intake.moveUp()));
    operatorRightTriggerR.whenActive(new InstantCommand(() -> m_intake.stopPivot()));
    operatorLeftTriggerR.whenActive(new InstantCommand(() -> m_intake.stopPivot()));

    //  Toggle hinge
    operatorLeftBumper.whenPressed(new ToggleHingeCmd(m_intake));

    // Run intake (In)
    operatorRightBumper.whileHeld(new ParallelCommandGroup(new RunCommand( () -> m_intake.take(-0.85)), new TakeBallCmd(m_indexer)));
    operatorRightBumper.whenInactive(new InstantCommand(() -> m_intake.take(0)));

    // Run indexer (In)
    operatorAButton.whenHeld(new RunCommand(() -> m_indexer.moveIndex()));
    operatorAButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));

    // Run indexer (Out)
    operatorYButton.whileHeld(new RunCommand(() -> m_indexer.reverseIndex()));
    operatorYButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));

    // Toggle shooter
    operatorBButton.whenPressed(new ToggleShooter(m_shooter));

    //--------------------------------------------------
    // Driver Controls!
    //--------------------------------------------------

    // Reset homge
    driverBButton.whenHeld(new RunCommand(() -> m_intake.moveUp()));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.stopPivot()));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.calibrate()));

    // Shuttle
    driverLeftBumper.whenHeld(new RunCommand(() -> m_climber.moveMotorsCounterClockwise()));
    driverRightBumper.whenHeld(new RunCommand(() -> m_climber.moveMotorsClockwise()));
    driverLeftBumper.whenInactive(new InstantCommand(() -> m_climber.stopMotors()));
    driverRightBumper.whenInactive(new InstantCommand(() -> m_climber.stopMotors()));


    // Vision
    driverAButton.whileHeld(new RunCommand(() -> m_Vision.getTargets()));

    driverLeftBumper.whenPressed(new ClimberAuton(m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
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

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
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
    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
    
  }

}



// Shooter and indexer don't turn off
// Pivot doesn't stop moving downward












//LOL