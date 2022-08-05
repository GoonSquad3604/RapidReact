// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private static Vision _instance;

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); 
  public boolean hasTarget = false;
  public double tv, ty, ta, tx;
  public double lastTY = 0;


  
  public Vision() {

  }
  public static final Vision getInstance() {
    if (_instance == null) {
            _instance = new Vision();
    }
    return _instance;
  } 
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Target", hasTarget);
    
    //SmartDashboard.putNumber("TX", tx);

    updateTargetInfo();
    SmartDashboard.putNumber("Target Distance", getDistance());
  }

  public void updateTargetInfo() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    hasTarget = tv > 0;
    if(ty != 0){
      lastTY = ty;
    }

  }

  public double getDistance() {
    
    double targetOffsetAngle_Vertical = ty != 0 ? ty : lastTY;

    //if(hasTarget) {      
      double angleToGoalDegrees = Constants.visionAngleDeg + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      //calculate distance
      double distanceFromLimeLight = (Constants.visionTargetHeight - Constants.visionHeight)/Math.tan(angleToGoalRadians);

      return distanceFromLimeLight; //meters  
    //}
    //else return 0;
  }


}

//69funny
