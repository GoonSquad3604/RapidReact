// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
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
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shuttle;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Aim;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.ClimberAuton;
import frc.robot.commands.ClimberAuton2;
import frc.robot.commands.ClimberAuton3;
import frc.robot.commands.ClimberAuton4;
import frc.robot.commands.DeployClimber;
import frc.robot.commands.FourBallAuton;
import frc.robot.commands.ShootAll;
import frc.robot.commands.TakeBallCmd;
import frc.robot.commands.ToggleHingeCmd;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.TwoBallAuton;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  //Subsystems
  private final Drivetrain m_driveTrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Index m_indexer = new Index();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber(); // LOLLLLLLLLLLLL 69
  private final Vision m_Vision = new Vision();
  private final Shuttle m_shuttle = new Shuttle();

  //Controllers
  XboxController m_driverController = new XboxController(0);
  XboxController m_operatorController = new XboxController(1);

  //Paths
  private String Auton5Ball1 = "paths/5ball1.wpilib.json";
  Trajectory Auton5BallTrajectory1 = new Trajectory();

  private String Auton5Ball2 = "paths/5ball2.wpilib.json";
  Trajectory Auton5BallTrajectory2 = new Trajectory();

  private String Auton5Ball3 = "paths/5ball3.wpilib.json";
  Trajectory Auton5BallTrajectory3 = new Trajectory();

  private String Auton4Ball1 = "paths/4ball1.wpilib.json";
  Trajectory Auton4BallTrajectory1 = new Trajectory();

  private String Auton4Ball2 = "paths/4ball2.wpilib.json";
  Trajectory Auton4BallTrajectory2 = new Trajectory();

  private String Auton4Ball3 = "paths/4ball3.wpilib.json";
  Trajectory Auton4BallTrajectory3 = new Trajectory();

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private TwoBallAuton twoBall; 
  private FourBallAuton fourBall;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putNumber("shooterVelo", 14500);
    //SmartDashboard.putNumber("shooter speed", 4000);
    SmartDashboard.putNumber("shooterPower", .5);
    SmartDashboard.putBoolean("isABot", Constants.isABot);

    m_driveTrain.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new RunCommand(
          () ->
              m_driveTrain.ArcadeDrive(
                  -m_driverController.getLeftY(), m_driverController.getRightX()),
          m_driveTrain)
      );

    configureButtonBindings();
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
    operatorRightBumper.whileHeld(new ParallelCommandGroup(new RunCommand( () -> m_intake.take(-0.8)), new TakeBallCmd(m_indexer)));
    operatorRightBumper.whenInactive(new InstantCommand(() -> m_intake.take(0)));

    // Run indexer (In)
    operatorAButton.whenHeld(new RunCommand(() -> m_indexer.moveIndex()));
    operatorAButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));

    // Run indexer (Out)
    operatorYButton.whileHeld(new RunCommand(() -> m_indexer.reverseIndex()));
    operatorYButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));

    // Toggle shooter
    operatorControlPadLeft.whenActive(new ToggleShooter(m_shooter, 14500));
    operatorXButton.whenPressed(new ShootAll(m_indexer, m_shooter));
    operatorControlPadUp.whenActive(new ToggleShooter(m_shooter, 18000));
    operatorControlPadDown.whenActive(new ToggleShooter(m_shooter, 8000));
    operatorBButton.whenPressed(new ToggleShooter(m_shooter, m_Vision));

    operatorRightStick.whenActive(new InstantCommand(() -> m_indexer.setBallCount0()));

    

    //--------------------------------------------------
    // Driver Controls!
    //--------------------------------------------------

    // Reset homge
    driverBButton.whenHeld(new RunCommand(() -> m_intake.moveUp()));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.stopPivot()));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.calibrate()));

    // Shuttle
    driverLeftBumper.whenHeld(new RunCommand(() -> m_shuttle.shuttleBack()));
    driverRightBumper.whenHeld(new RunCommand(() -> m_shuttle.shuttleForward()));
    driverLeftBumper.whenInactive(new InstantCommand(() -> m_shuttle.stopMotors()));
    driverRightBumper.whenInactive(new InstantCommand(() -> m_shuttle.stopMotors()));

    driverYButton.whenPressed(new DeployClimber(m_shuttle, m_climber));
    driverStartButton.whenPressed(new ClimberAuton(m_climber, m_shuttle, m_driveTrain));
    driverBackButton.whenPressed(new ClimberAuton2(m_climber, m_shuttle, m_driveTrain));
    driverXButton.whenPressed(new ClimberAuton3(m_climber, m_shuttle, m_driveTrain));
    driverAButton.whenPressed(new ClimberAuton4(m_climber, m_shuttle, m_driveTrain));


    //driverLeftTriggerP.whenActive(new AimAndShoot(m_Vision, m_shooter, m_driveTrain, m_indexer));
    driverLeftTriggerP.whenActive(new AimAndShoot(m_Vision, m_shooter, m_driveTrain, m_indexer));


    //operatorRightStick.whenHeld(new InstantCommand( () -> CommandScheduler.getInstance().cancelAll()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return new TwoBallAuton(m_driveTrain, Auton5BallTrajectory1, Auton5BallTrajectory2, Auton5BallTrajectory3, m_intake, m_indexer, m_shooter);
    return m_chooser.getSelected();
  }

  public void loadTrajectories() {
    try {
      Path Auton5Ball1Path = Filesystem.getDeployDirectory().toPath().resolve(Auton5Ball1);
      //Path Auton5Ball2Path = Filesystem.getDeployDirectory().toPath().resolve(Auton5Ball2);
      //Path Auton5Ball3Path = Filesystem.getDeployDirectory().toPath().resolve(Auton5Ball3);

      Path Auton4Ball1Path = Filesystem.getDeployDirectory().toPath().resolve(Auton4Ball1);
      Path Auton4Ball2Path = Filesystem.getDeployDirectory().toPath().resolve(Auton4Ball2);
      Path Auton4Ball3Path = Filesystem.getDeployDirectory().toPath().resolve(Auton4Ball3);

      Auton5BallTrajectory1 = TrajectoryUtil.fromPathweaverJson(Auton5Ball1Path);
      //Auton5BallTrajectory2 = TrajectoryUtil.fromPathweaverJson(Auton5Ball2Path);
      //Auton5BallTrajectory3 = TrajectoryUtil.fromPathweaverJson(Auton5Ball3Path);

      Auton4BallTrajectory1 = TrajectoryUtil.fromPathweaverJson(Auton4Ball1Path);
      Auton4BallTrajectory2 = TrajectoryUtil.fromPathweaverJson(Auton4Ball2Path);
      Auton4BallTrajectory3 = TrajectoryUtil.fromPathweaverJson(Auton4Ball3Path);
    } catch(IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " , ex.getStackTrace());
    }

    twoBall = new TwoBallAuton(m_driveTrain, Auton5BallTrajectory1, m_intake, m_indexer, m_shooter);

    fourBall = new FourBallAuton(m_driveTrain, Auton4BallTrajectory1, Auton4BallTrajectory2, Auton4BallTrajectory3, m_intake, m_indexer, m_shooter, m_Vision);

    m_chooser.setDefaultOption("TWO BALL AUTON", twoBall);
    m_chooser.addOption("FOUR BALL AUTON", fourBall);

    SmartDashboard.putData(m_chooser);

  }

  public void setCoastMode() {
    m_driveTrain.setCoastMode();
  }
}



// Shooter and indexer don't turn off
// Pivot doesn't stop moving downward


