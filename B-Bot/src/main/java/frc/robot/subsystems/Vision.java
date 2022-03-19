// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera pCamera = new PhotonCamera("photonvision");

  boolean hasTarget = false;
  
  public Vision() {
    pCamera.setPipelineIndex(0);
    pCamera.setDriverMode(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Target", hasTarget);

    getTargets();
    //SmartDashboard.putBoolean("drivermode", pCamera.getDriverMode());
  }

  public void getTargets() {
    var results = pCamera.getLatestResult();

    if(results.hasTargets()) {
      System.out.println("has targets");
    }

    else {
      System.out.println("nope");
    }

    
  }

  public void setDriverMode() {
    if(pCamera.getDriverMode()) {
      pCamera.setDriverMode(false);
    }
    else {
      pCamera.setDriverMode(true);

    }
  }
}
