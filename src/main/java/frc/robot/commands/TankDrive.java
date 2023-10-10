package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {

    private final Drivetrain m_drive;
    private final CommandXboxController m_joystick;

    public TankDrive(Drivetrain drivetrain, CommandXboxController joystick) {
        m_drive = drivetrain;
        m_joystick = joystick;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();
        double zRotation = -m_joystick.getLeftX();

        m_drive.arcadeDrive(xSpeed, zRotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return false;
    }

}
