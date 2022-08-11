package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class util {
    public static Command getPathPlannerSwerveControllerCommand(PathPlannerTrajectory traj) {
        return new PPSwerveControllerCommand(
                traj,
                DrivetrainSubsystem.getInstance()::getPose,
                DrivetrainSubsystem.getInstance().getKinematics(),
                new PIDController(8, 0, 0), 
                new PIDController(8, 0, 0), 
                new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.getAngularVelocity(), DrivetrainSubsystem.getAngularVelocity())),
                DrivetrainSubsystem.getInstance()::setStates,
                DrivetrainSubsystem.getInstance());
    }

    public static PathPlannerTrajectory twoBallAutonTrajectory() {
        return PathPlanner.loadPath(
                "twoballswervetop",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }

    public static PathPlannerTrajectory singleBallAutonTrajectory() {
        return PathPlanner.loadPath(
                "singleBallStraightOn",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }

    public static PathPlannerTrajectory fiveBall1Trajectory() {
        return PathPlanner.loadPath(
                "FiveBall1",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }

    public static PathPlannerTrajectory fiveBall2Trajectory() {
        return PathPlanner.loadPath(
                "FiveBall2",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }

    public static PathPlannerTrajectory fiveBall3Trajectory() {
        return PathPlanner.loadPath(
                "FiveBall3",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }

    public static PathPlannerTrajectory fiveBall4Trajectory() {
        return PathPlanner.loadPath(
                "FiveBall4",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }
}
