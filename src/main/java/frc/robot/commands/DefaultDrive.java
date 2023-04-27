package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

    Drivetrain sys_drivetrain;
    CommandXboxController m_controller;

    public DefaultDrive(Drivetrain drivetrain, CommandXboxController controller) {
        sys_drivetrain = drivetrain;
        m_controller = controller;

        addRequirements(drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed = -m_controller.getLeftY() * kDrive.kMaxDriveVelocity;
        double ySpeed = -m_controller.getLeftX() * kDrive.kMaxDriveVelocity;
        double rotation = m_controller.getRightX() * kDrive.kMaxDriveAngularVelocity;

        sys_drivetrain.drive(xSpeed, ySpeed, rotation);
    }
}
