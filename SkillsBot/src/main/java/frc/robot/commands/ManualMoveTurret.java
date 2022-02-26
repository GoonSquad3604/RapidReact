// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ManualMoveTurret extends CommandBase {
  private final Shooter m_shooter;
  private final DoubleSupplier m_supplier;

  /** Creates a new ManualMoveTurret. */
  public ManualMoveTurret(Shooter shooter, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_supplier = power;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setTurret(m_supplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
