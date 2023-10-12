// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kControllers;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.SwerveFallBacks.CheckTurnMotor;
import frc.robot.commands.SwerveFallBacks.ToggleIdleMode;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoTrajectorySelector;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static boolean testMode = true;

    // Joysticks
    private final CommandXboxController m_primaryController;
    private final CommandXboxController m_secondaryController;

    // Subsystems
    public final Drivetrain sys_drivetrain;

    // Commands
    private final SwerveDrive cmd_swerveDrive;
    private final TankDrive cmd_tankDrive;

    // Shuffleboard
    public final ShuffleboardTab sb_driveteamTab;

    // Autonomous
    private final AutoTrajectorySelector m_autoSelector;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Joysticks
        m_primaryController = new CommandXboxController(kControllers.kPrimaryController);
        m_secondaryController = new CommandXboxController(kControllers.kSecondaryController);

        // Subsystems
        sys_drivetrain = new Drivetrain();

        // Commands
        cmd_swerveDrive = new SwerveDrive(sys_drivetrain, m_primaryController);
        cmd_tankDrive = new TankDrive(sys_drivetrain, m_primaryController);

        sys_drivetrain.setDefaultCommand(new ConditionalCommand(cmd_tankDrive, cmd_swerveDrive, () -> sys_drivetrain.isTank()));

        // Shuffleboard
        sb_driveteamTab = Shuffleboard.getTab("Drive team");
        m_autoSelector = new AutoTrajectorySelector();
        addShuffleboardItems();

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_primaryController.leftBumper()
            .onTrue(Commands.runOnce(sys_drivetrain::resetAllEncoders));

        m_primaryController.rightBumper()
            .onTrue(Commands.runOnce(sys_drivetrain::zeroHeading, sys_drivetrain));

        m_primaryController.start()
            .onTrue(new CheckTurnMotor(sys_drivetrain));
        
        m_primaryController.back()
            .onTrue(new ToggleIdleMode(sys_drivetrain));

        if (testMode) {
            
            // Align wheels straight
            m_primaryController.x()
                .whileTrue(Commands.run(() -> sys_drivetrain.setTurnDegrees(0), sys_drivetrain));

        }
    }

    private void addShuffleboardItems() {

        // Re-zero
        sb_driveteamTab.add("Re-zero encoders", Commands.runOnce(sys_drivetrain::resetAllEncoders, sys_drivetrain))
            .withPosition(0, 0);
        sb_driveteamTab.add("Re-zero gyro",     Commands.runOnce(sys_drivetrain::zeroHeading, sys_drivetrain))
            .withPosition(1, 0);

        // Autonomous
        sb_driveteamTab.add("Choose auto", m_autoSelector.getSendableChooser())
            .withPosition(0, 1)
            .withSize(3, 1);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Get selected auto trajectory
        PathPlannerTrajectory chosenTrajectory = m_autoSelector.getChosenTrajectory();

        return new FollowTrajectory(chosenTrajectory, true, sys_drivetrain);
    }
}
