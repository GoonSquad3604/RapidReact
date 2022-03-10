// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ToggleShooter extends CommandBase {
  
  private Shooter m_shooter;
  private boolean fin;

  public ToggleShooter(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_shooter.isRunning) {
      m_shooter.setShooter(SmartDashboard.getNumber("shooterPower",0));
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