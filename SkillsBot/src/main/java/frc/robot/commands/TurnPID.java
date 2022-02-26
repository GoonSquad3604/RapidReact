// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnPID extends PIDCommand {
  
  private DriveTrain m_driveTrain; 
  /** Creates a new TurnPID. */
  public TurnPID(double angle, DriveTrain driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(4.5, 12, .75),
        // This should return the measurement
        driveTrain::getHeading,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        a -> driveTrain.arcadeDrive(0, -a));
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_driveTrain = driveTrain; 
    addRequirements(m_driveTrain);

    getController().setTolerance(1);

  }

  // Returns true when the command should end.
  @Override
  public void initialize() {
    m_driveTrain.reset();
    m_driveTrain.setCoastMode();
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setBrakeMode();
    //m_driveTrain.reset();
    m_driveTrain.arcadeDrive(0, 0);
    super.end(interrupted);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
