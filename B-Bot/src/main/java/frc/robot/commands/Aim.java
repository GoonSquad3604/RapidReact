package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class Aim extends CommandBase {
  
  private Vision m_vision;
  private Drivetrain m_drive;
  private double direction;
  private double speed;

  public Aim(Vision vision, Drivetrain drive) {
    m_vision = vision;
    m_drive = drive;
  }

  @Override
  public void initialize() {
    //m_drive.setBrakeMode();
    // if(!m_vision.hasTarget){
    //   end(false);
    // }    
    if(m_vision.tx > 0) direction = -1.0;
    else direction = 1.0;
  }

  @Override
  public void execute() {

    if(m_vision.hasTarget) {
      speed = Math.abs(m_vision.tx)/15.0;
      if(speed < 0.4) speed = 0.4;
    }
    else speed = 1;

    if(m_vision.tx > 0) direction = -1.0;
    else direction = 1.0;

    m_drive.TankDrive(-speed*direction, speed*direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.TankDrive(0, 0);
    //m_drive.setCoastMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_vision.tx) < 0.1 && m_vision.hasTarget) || !m_vision.hasTarget;
  }
}
