// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shuttle;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Aim;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AimSwerve;
import frc.robot.commands.AimAndShootTeleop;
import frc.robot.commands.ClimberAuton;
import frc.robot.commands.ClimberAuton2;
import frc.robot.commands.ClimberAuton3;
import frc.robot.commands.ClimberAuton4;
import frc.robot.commands.DeployClimber;
import frc.robot.commands.FiveBallAutonSwerve;
import frc.robot.commands.FourBallAuton;
import frc.robot.commands.GoonAutonCommand;
import frc.robot.commands.ShootAll;
import frc.robot.commands.SingleBallAutonSwerve;
import frc.robot.commands.TakeBallCmd;
import frc.robot.commands.ToggleHingeCmd;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.TwoBallAuton;
import frc.robot.commands.TwoBallAutonSwerve;
import frc.robot.commands.TwoBallBottomRightSwerve;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** // LOLLLLLLLLLLLL 69
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... 69 hahahahaha

  //Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
  //private final Drivetrain m_driveTrain = new Drivetrain();
  private final Intake m_intake = Intake.getInstance();
  private final Index m_indexer = Index.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climber m_climber = new Climber(); 
  private final Vision m_Vision = Vision.getInstance();
  //private final Shuttle m_shuttle = new Shuttle();

  //Controllers
  XboxController m_driverController = new XboxController(0);
  XboxController m_operatorController = new XboxController(1);

 

  private SendableChooser<GoonAutonCommand> autonChooser = new SendableChooser<GoonAutonCommand>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    configureButtonBindings();

    initAutonChooser();
  
  }

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
    final JoystickButton operatorRightStick = new JoystickButton(m_operatorController, XboxController.Button.kRightStick.value);
    final JoystickButton operatorLeftStick = new JoystickButton(m_operatorController, XboxController.Button.kLeftStick.value);
    // Operator Triggers
    final Trigger operatorRightTriggerP = new Trigger(() -> m_operatorController.getRightTriggerAxis() >= .5);
    final Trigger operatorLeftTriggerP = new Trigger(() -> m_operatorController.getLeftTriggerAxis() >= .5);
    final Trigger operatorRightTriggerR = new Trigger(() -> m_operatorController.getRightTriggerAxis() < .5);
    final Trigger operatorLeftTriggerR = new Trigger(() -> m_operatorController.getLeftTriggerAxis() < .5);

    //Operator POVs
    final Trigger operatorControlPadUp = new Trigger(() -> m_operatorController.getPOV() == 0);
    final Trigger operatorControlPadDown = new Trigger(() -> m_operatorController.getPOV() == 180);
    final Trigger operatorControlPadLeft = new Trigger(() -> m_operatorController.getPOV() == 270);


    // Driver joysticks
    final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    final JoystickButton driverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    final JoystickButton driverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    final JoystickButton driverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    final JoystickButton driverStartButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
    final JoystickButton driverBackButton = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

    final JoystickButton driverRightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    final JoystickButton driverLeftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

    final Trigger driverRightTriggerP = new Trigger(() -> m_driverController.getRightTriggerAxis() >= .5);
    final Trigger driverLeftTriggerP = new Trigger(() -> m_driverController.getLeftTriggerAxis() >= .5);
    final Trigger driverRightTriggerR = new Trigger(() -> m_driverController.getRightTriggerAxis() < .5);
    final Trigger driverLeftTriggerR = new Trigger(() -> m_driverController.getLeftTriggerAxis() < .5);

    //--------------------------------------------------
    // Operator Controls!
    //--------------------------------------------------
  
    // Telescope motors

    // Move hinge
    operatorRightTriggerP.whenActive(new InstantCommand(() -> m_intake.moveDown()));
    operatorLeftTriggerP.whenActive(new InstantCommand(() -> m_intake.moveUp()));
    operatorRightTriggerR.whenActive(new InstantCommand(() -> m_intake.stopPivot()));
    operatorLeftTriggerR.whenActive(new InstantCommand(() -> m_intake.stopPivot()));

    //  Toggle hinge
    operatorLeftBumper.whenPressed(new ToggleHingeCmd(m_intake));

    // Run intake (In)
    operatorRightBumper.whileHeld(new ParallelCommandGroup(new RunCommand( () -> m_intake.take(-0.8)), new TakeBallCmd(m_indexer)));
    operatorRightBumper.whenInactive(new InstantCommand(() -> m_intake.take(0)));

    // Run indexer (In)
    operatorAButton.whenHeld(new RunCommand(() -> m_indexer.moveIndex()));
    operatorAButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));

    // Run indexer (Out)
    operatorYButton.whileHeld(new RunCommand(() -> m_indexer.reverseIndex()));
    operatorYButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));

    // Toggle shooter%
    operatorControlPadLeft.whenActive(new ToggleShooter(m_shooter, 14500));
    operatorXButton.whenPressed(new ShootAll(m_indexer, m_shooter));
    operatorControlPadUp.whenActive(new ToggleShooter(m_shooter, 18000));
    operatorControlPadDown.whenActive(new ToggleShooter(m_shooter, 14400));
    operatorBButton.whenPressed(new ToggleShooter(m_shooter, 10000));

    operatorRightStick.whenActive(new InstantCommand(() -> m_indexer.setBallCount0()));
    operatorLeftStick.whenActive(new InstantCommand(()-> {m_indexer.incrementBallCount(); m_indexer.incrementBallCount();}));

    //--------------------------------------------------
    // Driver Controls!
    //--------------------------------------------------

    // Reset homge
    driverBButton.whenHeld(new RunCommand(() -> m_intake.moveUp(), m_intake));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.stopPivot(), m_intake));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.calibrate()));

    driverBackButton.whenHeld(new RunCommand(() -> m_climber.moveTelescopeUp(), m_climber));
    driverStartButton.whenHeld(new RunCommand(() -> m_climber.moveTelescopeDown(), m_climber));
    driverBackButton.whenInactive(new InstantCommand(() -> m_climber.stopTelescopeMotors(), m_climber));
    driverStartButton.whenInactive(new InstantCommand(() -> m_climber.stopTelescopeMotors(), m_climber));

    driverLeftTriggerP.whenActive(new AimAndShoot(m_Vision, m_shooter, m_drivetrainSubsystem, m_indexer));

    driverRightBumper.whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return new TwoBallAuton(m_driveTrain, Auton5BallTrajectory1, Auton5BallTrajectory2, Auton5BallTrajectory3, m_intake, m_indexer, m_shooter);
    //return null;
    m_drivetrainSubsystem.resetOdometry(autonChooser.getSelected().getInitialPose());  
    return autonChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
  
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);
  
    // Square the axis
    value = Math.copySign(value * value, value);
  
    return value;
  }

  private void initAutonChooser() {
    autonChooser.setDefaultOption("Two Ball Left - Starts left side pointed at ball and at the line.", new TwoBallAutonSwerve());
    autonChooser.addOption("Single Ball Straight - Starts in mid, parallel and facing driver station wall.", new SingleBallAutonSwerve());
    //autonChooser.addOption("Five Ball - Starts at right side, on line pointed at ball.", new FiveBallAutonSwerve());
    //autonChooser.addOption("Two Ball Right - Starts same position as five ball.", new TwoBallBottomRightSwerve());
    SmartDashboard.putData(autonChooser);
  }
  
}
// Shooter and indexer don't turn off
// Pivot doesn't stop moving downward


