// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AimSwerve extends CommandBase {

  private DrivetrainSubsystem m_drive;
  private Timer timer;
  private Vision m_vision;
  private double direction;
  private double speed;

  public AimSwerve(Vision vision, DrivetrainSubsystem drive) {
    m_vision = vision;
    m_drive = drive;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_vision.hasTarget){
      end(false);
    }    
    if(m_vision.tx > 0) direction = -1.0;
    else direction = 1.0;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_vision.hasTarget) {
      speed = Math.abs(m_vision.tx)/27.0;
      if(speed < 0.3) speed = 0.3;
    }
    //else speed = 1;

    if(m_vision.tx > 0) direction = -1.0;
    else direction = 1.0;

    m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, m_drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND*(speed/6.0)*direction, m_drive.getGyroscopeRotation()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drive.getGyroscopeRotation()));
    timer.stop();
    timer.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(m_vision.tx) < 0.1) || !m_vision.hasTarget || timer.get() > 1.75);
  }
}
