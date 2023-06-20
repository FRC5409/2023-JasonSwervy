package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.kAutonomous;

public class AutoTrajectorySelector {

    private PathPlannerTrajectory[] trajectories;
    private SendableChooser<PathPlannerTrajectory> trajectoryChooser;

    public AutoTrajectorySelector() {

        // Load trajectories
        trajectories = new PathPlannerTrajectory[kAutonomous.kTrajectories.length];
        loadTrajectories();

        // Choose trajectory
        trajectoryChooser = new SendableChooser<>();
        insertTrajectoriesIntoSendableChooser();

    }

    /**
     * Load all trajectories.
     * All trajectories are loaded on initialize, to prevent delay on enable.
     */
    private void loadTrajectories() {
        for (int i = 0; i < kAutonomous.kTrajectories.length; i++) {

            String trajectoryFilePath = kAutonomous.kTrajectories[i];
            PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFilePath, new PathConstraints(kAutonomous.kMaxDriveVelocity, kAutonomous.kMaxDriveAcceleration));

            trajectories[i] = trajectory;
        }
    }

    /**
     * Insert the trajectories into the SendableChooser.
     */
    private void insertTrajectoriesIntoSendableChooser() {
        for (int i = 0; i < kAutonomous.kTrajectories.length; i++) {

            String trajectoryName = kAutonomous.kTrajectories[i];
            PathPlannerTrajectory trajectory = trajectories[i];

            if (i == 0) {
                // set default option to first
                trajectoryChooser.setDefaultOption(trajectoryName, trajectory);
            } else {
                // add all other options
                trajectoryChooser.addOption(trajectoryName, trajectory);
            }
        }
    }

    public SendableChooser<PathPlannerTrajectory> getSendableChooser() {
        return trajectoryChooser;
    }

    /**
     * Get the chosen trajectory from the SendableChooser on Shuffleboard.
     * This is used in RobotContainer.java to create the autonomous command.
     * 
     * @return chosen trajectory
     */
    public PathPlannerTrajectory getChosenTrajectory() {
        return trajectoryChooser.getSelected();
    }
}
