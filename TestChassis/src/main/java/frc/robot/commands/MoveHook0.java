package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climbMotors;

public class MoveHook0 extends CommandBase {
  private climbMotors m_climber;
  private double encoderMotor0;
  private double encoderMotor1;
  boolean moveForward;

  /** Creates a new MoveHook0. */
  public MoveHook0(climbMotors climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveForward = false;
    encoderMotor0 = m_climber.getEncoderMotor0();
    encoderMotor1 = m_climber.getEncoderMotor1();

    if(encoderMotor0 > 0) moveForward = false;
    else moveForward = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(encoderMotor0 != 0) {
      if(moveForward) {
        m_climber.moveMotorsClockwise();
      }

      else {
        m_climber.moveMotorsCounterClockwise();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return encoderMotor0 == 0;
  }
}
