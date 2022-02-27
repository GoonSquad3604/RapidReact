// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.SetShooterToPowerCmd;
import frc.robot.commands.SetShooterToSpeedCmd;
import frc.robot.commands.TakeBallCmd;
import frc.robot.commands.ToggleHingeCmd;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  //private final Intake m_intake = new Intake();
  //private final Climber m_climbMotors = new Climber();
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

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Operator joysticks
    final JoystickButton operatorAButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
    final JoystickButton operatorYButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
    final JoystickButton operatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
    final JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
    final JoystickButton operatorXButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);

    // Driver joysticks
    final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);

    // Operator press buttons
    operatorRightBumper.whileHeld(new ParallelCommandGroup(new RunCommand( () -> m_intake.take(-0.85)), new TakeBallCmd(m_indexer)));
    operatorLeftBumper.whenPressed(new ToggleHingeCmd(m_intake));
    //operatorXButton.whenHeld(new RunCommand(() -> m_indexer.moveIndex()));
    operatorAButton.whenPressed(new ParallelCommandGroup(new SetShooterToPowerCmd(m_shooter), new RunCommand(() -> m_indexer.moveIndex())));
    operatorYButton.whenHeld(new RunCommand(() -> m_indexer.reverseIndex()));
    operatorXButton.whenHeld(new RunCommand(() -> m_intake.moveDown()));

    //operatorAButton.whileHeld(new RunCommand(() -> m_shooter.setShooter(.5)));


    // Driver press buttons
    driverBButton.whileHeld(new RunCommand(() -> m_intake.moveUp()));
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.calibrate()));

    // Stop pivot
    driverBButton.whenInactive(new RunCommand(() ->
      m_intake.stopPivot()));

    // Operator stop
    // operatorAButton.whenInactive(new InstantCommand(() -> m_shooter.setShooter(0)));
    operatorAButton.whenReleased(new ParallelCommandGroup(new InstantCommand(() -> m_shooter.setShooter(0), m_shooter), new InstantCommand(() -> m_indexer.stopIndex())));
    operatorYButton.whenInactive(new InstantCommand(() -> m_indexer.stopIndex()));
    operatorXButton.whenInactive(new InstantCommand(() -> m_intake.stopPivot()));

    operatorRightBumper.whenInactive(new InstantCommand(() -> m_intake.take(0)));
    
    operatorRightBumper.whenInactive(new RunCommand(() -> m_intake.take(0)));
    

    // Driver stop
    driverBButton.whenInactive(new InstantCommand(() -> m_intake.stopPivot()));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
