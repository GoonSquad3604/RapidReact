// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShoot extends SequentialCommandGroup {
  
  private Shooter m_shooter;
  private Vision m_vision;
  private Drivetrain m_driveTrain;
  private Index m_index;
  
  /** Creates a new AimAndShoot. */
  public AimAndShoot(Vision vision, Shooter shooter, Drivetrain driveTrain, Index index) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_shooter = shooter;
    m_vision = vision;
    m_driveTrain = driveTrain;
    m_index = index;

    double distance = m_vision.getDistance();
    double speed = m_shooter.getSpeedForDistance(distance);
    double angle = m_vision.tx;
    //set shooter based on distance
    // aim
    //shoot all+
    
    addCommands(
      new ToggleShooter(m_shooter, m_vision, true),
      new Aim(m_vision, m_driveTrain),
      new ShootAll(m_index, m_shooter),
      new ToggleShooter(m_shooter, m_vision)
      );
  }
}
