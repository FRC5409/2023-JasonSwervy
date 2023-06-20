package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends SequentialCommandGroup {

    /**
     * Follow a trajectory (path) autonomously.
     * 
     * @param trajectory trajectory to follow
     * @param isFirstPath is the first path (will reset odometry)
     * @param sys_drivetrain drivetrain subsystem
     */
    public FollowTrajectory(PathPlannerTrajectory trajectory, boolean isFirstPath, Drivetrain sys_drivetrain) {
        addCommands(

            // Reset odometry, if first path
            new InstantCommand(() -> {
                if (isFirstPath)
                    sys_drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
            }),

            new PPSwerveControllerCommand(
                trajectory,
                sys_drivetrain::getPose,
                sys_drivetrain.getKinematics(),
                new PIDController(kDrive.kDriveP, kDrive.kDriveI, kDrive.kDriveD), // X controller
                new PIDController(kDrive.kDriveP, kDrive.kDriveI, kDrive.kDriveD), // Y Controller
                new PIDController(kDrive.kTurnP, kDrive.kTurnI, kDrive.kTurnD), // Rotation controller
                sys_drivetrain::setModulesStates,
                sys_drivetrain // require Drivetrain
            )
            
        );
    }
}
