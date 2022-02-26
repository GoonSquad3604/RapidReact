// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexerMoveThrough extends CommandBase {
  private Indexer m_indexer;
  private boolean detected = false;
  /** Creates a new IndexerMoveThrough. */
  public IndexerMoveThrough(Indexer indexer) {
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
      m_indexer.runIndexer();
      
      if(!m_indexer.detectBall())
      {
        System.out.println("NO BALL");
          m_indexer.stop();
          detected = false;
      }
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
