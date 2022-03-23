// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera pCamera = new PhotonCamera("photonvision");

  boolean hasTarget = false;
  
  public Vision() {
    pCamera.setPipelineIndex(0);
    pCamera.setDriverMode(true);
    pCamera.setLED(VisionLEDMode.kDefault);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Target", hasTarget);

    //getTargets();
  }

  public void getTargets() {
    var results = pCamera.getLatestResult();
    //SmartDashboard.putBoolean("targetCheck", test );
    //SmartDashboard.putString("photonVision", pCamera.getLatestResult().toString());
    //SmartDashboard.putString("photontargets", results.getBestTarget().toString());
    System.out.println("pipeline  " + pCamera.getPipelineIndex());
    if(results.hasTargets()) {
      var x = results.getBestTarget().getYaw();
      hasTarget = true;
      System.out.println(x);
      SmartDashboard.putString("sup", results.getBestTarget().toString());
    }

    else {
      System.out.println("nope");
      hasTarget = false;
    }


  }

  public double getDistance() {
    return 4;
  }

  public void setDriverMode() {
    if(pCamera.getDriverMode()) {
      pCamera.setDriverMode(false);
    }
    else {
      pCamera.setDriverMode(true);

    }
  }

  public PhotonCamera getPhotonCamera() {
    return pCamera;
  }
}
