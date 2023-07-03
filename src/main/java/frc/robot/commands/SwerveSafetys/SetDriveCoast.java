package frc.robot.commands.SwerveSafetys;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SetDriveCoast extends InstantCommand {

    private final Drivetrain m_drive;

    public SetDriveCoast(Drivetrain drive) {
        m_drive = drive;        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.setDriveBrakeMode(false);
    }

}
