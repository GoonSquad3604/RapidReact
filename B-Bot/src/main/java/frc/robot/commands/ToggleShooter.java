// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ToggleShooter extends CommandBase {
  
  private Vision m_vision;
  private boolean hasVision;
  private Shooter m_shooter;
  private boolean fin;
  private double m_velocity;
  private boolean m_override;

  public ToggleShooter(Shooter shooter) {
    m_shooter = shooter;
    m_velocity = SmartDashboard.getNumber("shooterVelo",6000);
    //m_power = SmartDashboard.getNumber("shooterPower",0);
    m_override = false;
    addRequirements(shooter);
    hasVision = false;
  }

  public ToggleShooter(Shooter shooter, double velocity) {
    m_shooter = shooter;
    m_velocity = velocity;
    m_override = false;
    addRequirements(shooter);
    hasVision = false;
  }
  public ToggleShooter(Shooter shooter, double velocity, boolean override) {
    m_shooter = shooter;
    m_velocity = velocity;
    m_override = override;
    addRequirements(shooter);
    hasVision = false;
  }

  public ToggleShooter(Shooter shooter, Vision vision) {
    m_shooter = shooter;
    m_vision = vision;
    hasVision = true;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_shooter.isRunning || m_override) {
      if(hasVision) {
        double distance = m_vision.getDistance();
        m_velocity = m_shooter.getSpeedForDistance(distance);
      } 
      m_shooter.setShooterVelo(m_velocity);
      //m_shooter.setShooter(m_power);
      m_shooter.isRunning = true;
      //System.out.println("toggle on");
    }
    else {
      m_shooter.setShooter(0);
      m_shooter.isRunning = false;
      System.out.println("toggle off");
    }

    //System.out.println(m_shooter.isRunning);
    fin = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }
}
