package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class SetpointDrive extends CommandBase {

    private final Drivetrain m_drive;
    private final CommandXboxController m_joystick;
    private final PIDController m_headingController;

    private SlewRateLimiter m_xSpeedSlewRateLimiter;
    private SlewRateLimiter m_ySpeedSlewRateLimiter;

    public SetpointDrive(Drivetrain drive, CommandXboxController joystick) {
        m_drive = drive;
        m_joystick = joystick;

        m_xSpeedSlewRateLimiter = new SlewRateLimiter(kDrive.kXSpeedSlewRate);
        m_ySpeedSlewRateLimiter = new SlewRateLimiter(kDrive.kYSpeedSlewRate);

        // Robot heading PID
        m_headingController = new PIDController(kDrive.kHeadingP, kDrive.kHeadingI, kDrive.kHeadingD);
        m_headingController.setTolerance(Math.toRadians(3));
        m_headingController.enableContinuousInput(0, Math.toRadians(360));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Controller inputs
        double xSpeed = m_joystick.getLeftY();
        double ySpeed = m_joystick.getLeftX();

        xSpeed = m_xSpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(xSpeed, kDrive.kXSpeedDeadband));
        ySpeed = m_ySpeedSlewRateLimiter.calculate(MathUtil.applyDeadband(ySpeed, kDrive.kYSpeedDeadband));

        double xRotation = m_joystick.getRightX();
        double yRotation = m_joystick.getRightY();

        // deadband for target angle
        if (Math.abs(xRotation) < 0.2) xRotation = 0;
        if (Math.abs(yRotation) < 0.2) yRotation = 0;

        double targetAngle = getRotationTargetAngle(xRotation, yRotation);

        m_headingController.setSetpoint(targetAngle);

        double rotation = m_headingController.calculate(Math.toRadians(m_drive.getHeading()));

        m_drive.drive(xSpeed, ySpeed, rotation);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_joystick.getLeftTriggerAxis() + m_joystick.getRightTriggerAxis() != 0 || m_headingController.atSetpoint();
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
