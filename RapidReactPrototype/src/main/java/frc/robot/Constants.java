// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        //DriveTrain
        public static final int kLeftRear = 15;
        public static final int kLeftFront = 14;
        public static final int kRightRear = 30;
        public static final int kRightFront = 1;

        // Indexer
        public static final int kIndexMotorId = 5;
        public static final int kIndexMotor1Id = 7;
        public static final int kDigitalInputId = 5;

        //Shooter
        public static final int kTurretId = 4;
        public static final int kShooterMotor0Id = 2;
        public static final int kShooterMotor1Id = 3;

        //Intake
        public static final int kPivotMotorId = 10;
        public static final int kIntakeMotor0Id = 11;
        public static final int kIntakeMotor1Id = 6;
        public static final int kHingeEncoderA = 0;
        public static final int kHingeEncoderB = 1;
        public static final int kHingeEncoderIndex = 2;
        
        //Climber
        public static final int kTelescopeMotor0Id = 17;
        public static final int kTelescopeMotor1Id = 18;
        public static final int kShuttleMotor0Id = 19;
        public static final int kShuttleMotor1Id = 20;
}
