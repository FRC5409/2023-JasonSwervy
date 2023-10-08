package frc.robot.commands.SwerveFallBacks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ToggleIdleMode extends InstantCommand {

    private final Drivetrain m_drive;;

    public ToggleIdleMode(Drivetrain drivetrain) {
        m_drive = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.setBrakeMode(!m_drive.getBrakeMode());
    }

}
