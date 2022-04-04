// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class TakeBallCmd extends CommandBase {
  private Index m_indexer;
  private boolean detected = false;
  /** Creates a new TakeBallCmd. */
  public TakeBallCmd(Index indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;

    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_indexer.detectBall() && !detected)
    {
        detected = true;
        System.out.println("DING!");
    }
    
    if(detected) {
      System.out.println("The ball has been seen");

      if(m_indexer.getBallCount() == 0) {
        m_indexer.moveIndexAuto();
        if(m_indexer.detectExit())
        {
          m_indexer.incrementBallCount();
          //System.out.println("NO BALL");
          m_indexer.stopIndex();
          detected = false;
        }
      }
      else if(m_indexer.getBallCount() >= 1) {
        
        m_indexer.moveButtomIndex();
      
        if(!m_indexer.detectBall()) {
          m_indexer.incrementBallCount();
          System.out.println();
          m_indexer.stopIndex();
          detected = false;
        }
      }
      
      
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
