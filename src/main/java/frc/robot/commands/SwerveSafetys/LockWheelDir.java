package frc.robot.commands.SwerveSafetys;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.MathUtiles;

public class LockWheelDir extends InstantCommand {

    private final Drivetrain m_drive;
    private final CommandXboxController m_joystick;

    private double angle;

    public LockWheelDir(Drivetrain drive, CommandXboxController joystick) {
        m_drive = drive;
        m_joystick = joystick;        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        angle = MathUtiles.getRotation(m_joystick.getLeftX(), m_joystick.getLeftY());

        m_drive.setTurnAngle(Math.toRadians(angle));
        m_drive.setTank();
    }

}
