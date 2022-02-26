// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BasicAuton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IndexerMoveThrough;
import frc.robot.commands.ManualMoveTurret;
import frc.robot.commands.ResetTurret;
import frc.robot.commands.ShootPower;
import frc.robot.commands.ShooterToSetPower;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  XboxController m_driverController = new XboxController(0);
  XboxController m_operatorController = new XboxController(1);


  private final BasicAuton m_autoCommand = new BasicAuton(m_driveTrain, m_intake, m_indexer, m_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Setup Dashboard Varibles
    SmartDashboard.putNumber("shooter speed", 1600);


    // Configure the button bindings
    

    m_driveTrain.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new RunCommand(
          () ->
              m_driveTrain.arcadeDrive(
                  -m_driverController.getLeftY(), m_driverController.getRightX()),
          m_driveTrain)
      );

    //m_shooter.setDefaultCommand(new ManualMoveTurret(m_shooter, m_operatorController::getLeftX));

    configureButtonBindings();
    
  }

  /*
  This Function is meant to be used to set Coast mode at the end of auton and the beginning of Teleop.
  */
  public void setDriveTrainCoast(){
    m_driveTrain.setCoastMode();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton operatorAButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
    final JoystickButton operatorStartButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);  
    final JoystickButton operatorselectButton = new JoystickButton(m_operatorController, XboxController.Button.kBack.value);  
    final JoystickButton operatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
    final JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
    final JoystickButton operatorBButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
    final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    final JoystickButton operatorYButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);

    // Intake
    operatorLeftBumper.whenPressed(new ToggleIntake(m_intake));

    // Calibrate Intake
    operatorBButton.whileActiveOnce(new RunCommand(() ->
      m_intake.calibrate()
      ));

    driverBButton.whileHeld(new RunCommand(() ->
      m_intake.moveUp()));

    driverBButton.whenInactive(new InstantCommand(() ->
    m_intake.calibrate()
    ));

  // Stop pivot
    driverBButton.whenInactive(new RunCommand(() ->
      m_intake.stopPivot()));

    operatorYButton.whileHeld(new RunCommand(() ->
      m_indexer.runIndexer()));
    
    operatorYButton.whenInactive(new RunCommand(() ->
      m_indexer.stop()));


    //operatorAButton.whenPressed(new ParallelCommandGroup(new ShootPower(m_shooter,.4 ), new RunCommand(() -> m_indexer.runIndexer())));
    operatorAButton.whenPressed(new ParallelCommandGroup(new ShooterToSetPower(m_shooter), new RunCommand(() -> m_indexer.runIndexer())));

    operatorAButton.whenReleased(new ParallelCommandGroup(new ShootPower(m_shooter,0 ), new RunCommand(() -> m_indexer.stop())));
    operatorStartButton.whenPressed(new ResetTurret(m_shooter));

    operatorselectButton.whileHeld(new ManualMoveTurret(m_shooter, m_operatorController::getLeftX));  

    operatorRightBumper.whileHeld(new ParallelCommandGroup(new RunCommand( () ->
      m_intake.take()), new IndexerMoveThrough(m_indexer)
    ));



    operatorRightBumper.whenInactive(new RunCommand( () ->
      m_intake.stop()
    ));
    
    

    //final Trigger leftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() >= .5);

    //leftTrigger.whenActive(new RunCommand( () -> m_intake.moveUp()));
    
    //leftTrigger.debounce(.5).whenInactive(new RunCommand( () -> m_intake.stopPivot()));

    
    //final Trigger rightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() >= .5);

    //rightTrigger.whenActive(new RunCommand( () -> m_intake.moveDown()));
    
    //rightTrigger.and(leftTrigger).debounce(.5).whenInactive(new RunCommand( () -> m_intake.stopPivot()));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
