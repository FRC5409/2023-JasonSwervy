package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class ManualDrive extends CommandBase {

    Drivetrain sys_drivetrain;
    CommandXboxController m_controller;

    SlewRateLimiter m_xSpeedSlewRateLimiter;
    SlewRateLimiter m_ySpeedSlewRateLimiter;
    SlewRateLimiter m_manualRotationSlewRateLimiter;

    public ManualDrive(Drivetrain drivetrain, CommandXboxController controller) {
        sys_drivetrain = drivetrain;
        m_controller = controller;

        m_xSpeedSlewRateLimiter = new SlewRateLimiter(kDrive.kXSpeedSlewRate);
        m_ySpeedSlewRateLimiter = new SlewRateLimiter(kDrive.kYSpeedSlewRate);
        m_manualRotationSlewRateLimiter = new SlewRateLimiter(kDrive.kManualRotationSlewRate);

        addRequirements(drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Controller inputs
        double xSpeed = m_controller.getLeftY();
        double ySpeed = m_controller.getLeftX();

        double manualXRotation = -m_controller.getLeftTriggerAxis();
        double manualYRotation = m_controller.getRightTriggerAxis();

        double manualRotation = manualXRotation + manualYRotation;

        // Apply deadband and slew rate
        xSpeed = m_xSpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(xSpeed, kDrive.kXSpeedDeadband));
        ySpeed = m_ySpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(ySpeed, kDrive.kYSpeedDeadband));
        manualRotation = m_manualRotationSlewRateLimiter.calculate(MathUtil.applyDeadband(manualRotation, kDrive.kManualRotationDeadband));

        // Multiply/scale from percentage to speed
        xSpeed *= kDrive.kMaxDriveVelocity; // metres per second
        ySpeed *= kDrive.kMaxDriveVelocity; // metres per second
        manualRotation *= (kDrive.kMaxTurnAngularVelocity / 2); // radians

        sys_drivetrain.drive(xSpeed, ySpeed, manualRotation);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double rightX = m_controller.getRightX();
        double rightY = m_controller.getRightY();

        return rightX > kDrive.kXSpeedDeadband || rightY > kDrive.kYSpeedDeadband;
    }

}
