package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

    Drivetrain sys_drivetrain;
    CommandXboxController m_controller;

    double targetAngle;

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

        double xRotation = m_controller.getRightX();
        double yRotation = m_controller.getRightY();

        double manualXRotation = -m_controller.getLeftTriggerAxis();
        double manualYRotation = m_controller.getRightTriggerAxis();

        if (Math.abs(xSpeed) < 0.125) xSpeed = 0;
        if (Math.abs(ySpeed) < 0.125) ySpeed = 0;
        if (Math.abs(xRotation) < 0.2) xRotation = 0;
        if (Math.abs(yRotation) < 0.2) yRotation = 0;
        if (Math.abs(manualXRotation) < 0.1) manualXRotation = 0;
        if (Math.abs(manualYRotation) < 0.1) manualYRotation = 0;

        double manualRotation = manualXRotation + manualYRotation;

        targetAngle = getRotationTargetAngle(xRotation, yRotation);
        SmartDashboard.putNumber("target angle", Math.toDegrees(targetAngle));

        // Apply deadband and slew rate
        // xSpeed = m_xSpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(xSpeed, kDrive.kXSpeedDeadband));
        // ySpeed = m_ySpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(ySpeed, kDrive.kYSpeedDeadband));
        // rotation = m_rotationSlewRateLimiter.calculate(MathUtil.applyDeadband(rotation, kDrive.kRotationDeadband));

        // Multiply/scale from percentage to speed
        xSpeed *= kDrive.kMaxDriveVelocity; // metres per second
        ySpeed *= kDrive.kMaxDriveVelocity; // metres per second
        manualRotation *= kDrive.kMaxTurnAngularVelocity; // radians

        sys_drivetrain.drive(xSpeed, ySpeed, targetAngle, manualRotation);
    }

    /**
     * Get the target rotation angle, in radians.
     * 
     * 0 deg = forward
     * 90 deg = left
     * 180 deg = backward
     * 270 deg = right
     * 
     * @param xRotation right stick x
     * @param yRotation right stick y
     * @return target angle
     */
    private double getRotationTargetAngle(double xRotation, double yRotation) {

        if (xRotation == 0 && yRotation == 0)
            return targetAngle;
        
        double angle = Math.atan2(-xRotation, -yRotation);
        if (angle < 0) angle += Math.toRadians(360);
        return Math.round(angle / kDrive.kHeadingSnap) * kDrive.kHeadingSnap;
    }
}
