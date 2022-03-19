// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    
    //Drivetrain IDs
    public static final int kFrontLeftId = 1;
    public static final int kBackLeftId = 18;
    public static final int kFrontRightId = 19;
    public static final int kBackRightId = 40;
    public static final int kCANCoderRightId = 21;
    public static final int kCANCoderLeftId = 20;
    public static final int kPulsesPerMeter = 45000;

    //Index IDs
    public static final int kIndexMotorId = 15;
    public static final int kIndexMotor1Id = 16;
    public static final int kSensor0Id = 0;
    public static final int kSensor1Id = 1;


    //Shooter IDs
    public static final int kShooterMotor0Id = 17;
    public static final int kShooterMotor1Id = 2;

    //Intake IDs
    public static final int kPivotId = 6;
    public static final int kIntakeFrontId = 4;
    
    public static final int kHingeEncoderA = 7;
    public static final int kHingeEncoderB = 8;
    public static final int kHingeEncoderIndex = 9;
    public static final int kHingeBottomPosition = -700;
    
    //Climber IDs
    public static final int kTelescopeMotorRightId = 13;
    public static final int kTelescopeMotorLeftId = 5;
    public static final int kShuttleMotorRightId = 14;
    public static final int kShuttleMotorLeftId = 3;

    // Climber Positions
    public static final double kShuttleCenter = 0;
    public static final double kShuttleFront = 100;
    public static final double kShuttleBack = -80;

    // Auton
    public static final double ksVolts = 0.72609;
    public static final double kvVoltSecondsPerMeter = 0.46945;
    public static final double kaVoltSecondsSquaredPerMeter = 0.16887;
    public static final double kPDriveVel = 0.73025;
    
    public static final double kTrackwidthMeters = 0.523875;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3.624072;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Climber Auton
    public static final double leftTelescopeFull = 439.27;
    public static final double rightTelescopeFull = -431.45;
    public static final double leftOnHook = 389.25;
    public static final double rightOnHook = -377.81;
    public static final double leftPulled = 3.45;
    public static final double rightPulled = -2.43;

    public static final double leftShuttleFront = 153.21;
    public static final double rightShuttleFront = -153.49;
    public static final double leftShuttleBack = -172.35;
    public static final double rightShuttleBack = 171.23;
    public static final boolean isABot = false;







}
