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
                new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.getInstance().getAngularVelocity(), DrivetrainSubsystem.getInstance().getAngularVelocity())),
                DrivetrainSubsystem.getInstance()::setStates,
                DrivetrainSubsystem.getInstance());
    }

    public static PathPlannerTrajectory twoBallAutonTrajectory() {
        return PathPlanner.loadPath(
                "servemid1",
                DrivetrainSubsystem.getMaxVelocity(),
                4);
    }
}
