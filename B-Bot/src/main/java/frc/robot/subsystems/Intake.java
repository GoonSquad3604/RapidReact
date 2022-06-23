// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ConstantsA;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
 
  private CANSparkMax intakeFront = new CANSparkMax(Constants.kIntakeFrontId, MotorType.kBrushless);
  //private CANSparkMax pivot = new CANSparkMax(Constants.kPivotId, MotorType.kBrushless);
  private WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.kPivotId);

  //private CANSparkMax intakeRear = new CANSparkMax(Constants.kIntakeRear, MotorType.kBrushless);

  private int hingeTop;
  private int hingeBottom;
  public boolean isToggled = false;
  
  private Encoder hingeEncoder = new Encoder(Constants.kHingeEncoderA, Constants.kHingeEncoderB, Constants.kHingeEncoderIndex);
  
  public Intake() {

    // pivot.configFactoryDefault();
    // hingeEncoder.setReverseDirection(true);

    // pivot.configNominalOutputForward(0, Constants.kTimeoutMs);
		//   pivot.configNominalOutputReverse(0, Constants.kTimeoutMs);
		//   pivot.configPeakOutputForward(1, Constants.kTimeoutMs);
		//   pivot.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		//   pivot.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		// /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    //   pivot.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Hinge.kF, Constants.kTimeoutMs);
    //   pivot.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Hinge.kP, Constants.kTimeoutMs);
    //   pivot.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Hinge.kI, Constants.kTimeoutMs);
    //   pivot.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Hinge.kD, Constants.kTimeoutMs);
    //   pivot.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 
    //                                         Constants.kPIDLoopIdx,
		// 		                            Constants.kTimeoutMs);
      

    calibrate();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Hinge Encoder", hingeEncoder.get());

  }

  public void calibrate() {
    hingeEncoder.reset();
    hingeTop = 0;
    hingeBottom = Constants.isABot ? ConstantsA.kHingeBottomPositionA : Constants.kHingeBottomPosition;

    System.out.println("encoder: " + hingeEncoder.get());
    System.out.println("Position: " + getHingePosition().toString());
  }

  public void moveUp() {
    pivot.set(-0.85);
    //System.out.println("move up");
  }

  public void moveDown() {
    pivot.set(0.85);
  }

  public void moveUpAuto() {
    pivot.set(-0.85);
    //System.out.println("move up");
  }

  public void moveDownAuto() {
    pivot.set(0.85);
  }
  
  public void stopPivot() {
    pivot.set(0);
    //System.out.println("stopping");
  }

  public void take(double speed) {
    intakeFront.set(-speed);

    //System.out.println(speed);
    //intakeRear.set(speed);
  }

  public void vomit(double speed) {
    intakeFront.set(speed);
  }

  public void setHingePosition(double position) {
    pivot.set(TalonSRXControlMode.Position, position );
  }

  public HingePosition getHingePosition() {
    if(hingeEncoder.get() > hingeTop-20 ) {
      return HingePosition.Up;
    }

    else if(hingeEncoder.get() < hingeBottom+40) {
      return HingePosition.Down;
    }

    return HingePosition.Between;

  }
}