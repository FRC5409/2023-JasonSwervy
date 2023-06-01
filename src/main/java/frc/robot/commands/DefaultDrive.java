package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

    Drivetrain sys_drivetrain;
    CommandXboxController m_controller;

    // SlewRateLimiter m_xSpeedSlewRateLimiter;
    // SlewRateLimiter m_ySpeedSlewRateLimiter;
    // SlewRateLimiter m_rotationSlewRateLimiter;

    public DefaultDrive(Drivetrain drivetrain, CommandXboxController controller) {
        sys_drivetrain = drivetrain;
        m_controller = controller;

        // m_xSpeedSlewRateLimiter = new SlewRateLimiter(kDrive.kMaxDriveVelocity);
        // m_ySpeedSlewRateLimiter = new SlewRateLimiter(kDrive.kMaxDriveVelocity);
        // m_rotationSlewRateLimiter = new SlewRateLimiter(kDrive.kMaxTurnAngularAcceleration);

        addRequirements(drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Controller inputs
        double xSpeed = m_controller.getLeftY();
        double ySpeed = m_controller.getLeftX();
        double rotation = m_controller.getRightX();

        if (Math.abs(xSpeed) < 0.1) xSpeed = 0;
        if (Math.abs(ySpeed) < 0.1) ySpeed = 0;
        if (Math.abs(rotation) < 0.1) rotation = 0;

        // Apply deadband and slew rate
        // xSpeed = m_xSpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(xSpeed, kDrive.kXSpeedDeadband));
        // ySpeed = m_ySpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(ySpeed, kDrive.kYSpeedDeadband));
        // rotation = m_rotationSlewRateLimiter.calculate(MathUtil.applyDeadband(rotation, kDrive.kRotationDeadband));

        // Multiply/scale from percentage to speed
        xSpeed *= kDrive.kMaxDriveVelocity; // metres per second
        ySpeed *= kDrive.kMaxDriveVelocity; // metres per second
        rotation *= kDrive.kMaxDriveAngularVelocity; // radians per second

        sys_drivetrain.drive(xSpeed, ySpeed, rotation);
    }
}
