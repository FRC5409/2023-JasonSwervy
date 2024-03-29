package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {

    Drivetrain sys_drivetrain;
    CommandXboxController m_controller;

    SlewRateLimiter m_xSpeedSlewRateLimiter;
    SlewRateLimiter m_ySpeedSlewRateLimiter;
    SlewRateLimiter m_manualRotationSlewRateLimiter;

    public SwerveDrive(Drivetrain drivetrain, CommandXboxController controller) {
        sys_drivetrain = drivetrain;
        m_controller = controller;

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
        // deadband for target angle
        if (Math.abs(xRotation) < kDrive.kTargetHeadingDeadband) xRotation = 0;
        if (Math.abs(yRotation) < kDrive.kTargetHeadingDeadband) yRotation = 0;

        double manualXRotation = -m_controller.getLeftTriggerAxis();
        double manualYRotation = m_controller.getRightTriggerAxis();

        double manualRotation = manualXRotation + manualYRotation;

        double targetAngle = getRotationTargetAngle(xRotation, yRotation);

        // Apply deadband and slew rate
        xSpeed = MathUtil.applyDeadband(xSpeed, kDrive.kXSpeedDeadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, kDrive.kYSpeedDeadband);
        manualRotation = MathUtil.applyDeadband(manualRotation, kDrive.kManualRotationDeadband);

        // Multiply/scale from percentage to speed
        xSpeed *= kDrive.kMaxDriveVelocity; // metres per second
        ySpeed *= kDrive.kMaxDriveVelocity; // metres per second
        manualRotation *= (kDrive.kMaxTurnAngularVelocity); // radians

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
            return -1;
        
        double angle = Math.atan2(-xRotation, -yRotation);
        if (angle < 0) angle += Math.toRadians(360);
        return Math.round(angle / kDrive.kHeadingSnap) * kDrive.kHeadingSnap;
    }
}
