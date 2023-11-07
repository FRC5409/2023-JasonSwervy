package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.Location;
import frc.robot.Constants.kDrive.kCANCoder;
import frc.robot.Constants.kFallBack;
import frc.robot.Constants.kRobot;

public class Drivetrain extends SubsystemBase {

    // Swerve modules
    private final SwerveModule mod_frontLeft;
    private final SwerveModule mod_frontRight;
    private final SwerveModule mod_backLeft;
    private final SwerveModule mod_backRight;

    // Swerve kinematic points
    private final Translation2d m_frontLeftLoc;
    private final Translation2d m_frontRightLoc;
    private final Translation2d m_backLeftLoc;
    private final Translation2d m_backRightLoc;

    // Sensors and location
    private final WPI_Pigeon2 m_gyro;
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDriveOdometry m_odometry;

    private final Field2d m_field;

    // Robot heading PID
    private final PIDController m_headingController;
    private double headingControllerSetpoint = -1; // defaulted to -1 to start without a setpoint

    // Shuffleboard
    private boolean debugMode = true;
    private ShuffleboardTab sb_drivetrainTab;

    private boolean isTank = false;
    private DifferentialDrive m_diffDrive;
    

    public Drivetrain() {

        // Swerve modules
        mod_frontLeft   = new SwerveModule(kCANID.kDriveMotor1, kCANID.kTurnMotor1, kCANID.kCANCoder1, kCANCoder.kAbsoluteEncoderOffset1, true, true, Location.TopLeft);
        mod_frontRight  = new SwerveModule(kCANID.kDriveMotor2, kCANID.kTurnMotor2, kCANID.kCANCoder2, kCANCoder.kAbsoluteEncoderOffset2, false, true, Location.TopRight);
        mod_backLeft    = new SwerveModule(kCANID.kDriveMotor3, kCANID.kTurnMotor3, kCANID.kCANCoder3, kCANCoder.kAbsoluteEncoderOffset3, true, true, Location.BottomLeft);
        mod_backRight   = new SwerveModule(kCANID.kDriveMotor4, kCANID.kTurnMotor4, kCANID.kCANCoder4, kCANCoder.kAbsoluteEncoderOffset4, false, true, Location.BottomRight);

        // Swerve kinematic points
        m_frontLeftLoc  = new Translation2d(kRobot.length / 2, kRobot.width / 2);
        m_frontRightLoc = new Translation2d(kRobot.length / 2, -kRobot.width / 2);
        m_backLeftLoc   = new Translation2d(-kRobot.length / 2, kRobot.width / 2);
        m_backRightLoc  = new Translation2d(-kRobot.length / 2, -kRobot.width / 2);

        // Sensors and location
        m_gyro          = new WPI_Pigeon2(kCANID.kGyro);
        m_gyro.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ);
        zeroHeading();
        m_kinematics    = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);
        m_odometry      = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(),
                            new SwerveModulePosition[] {mod_frontLeft.getPosition(), mod_frontRight.getPosition(), mod_backLeft.getPosition(), mod_backRight.getPosition()});
        
        m_field = new Field2d();

        // Robot heading PID
        m_headingController = new PIDController(kDrive.kHeadingP, kDrive.kHeadingI, kDrive.kHeadingD);
        m_headingController.setTolerance(Math.toRadians(3));
        m_headingController.enableContinuousInput(0, Math.toRadians(360));

        // Shuffleboard
        if (debugMode) {
            sb_drivetrainTab = Shuffleboard.getTab("Drivetrain");
            sb_drivetrainTab.addNumber("VEL Front left",       () -> mod_frontLeft.getState().speedMetersPerSecond)          .withPosition(0, 0);
            sb_drivetrainTab.addNumber("VEL Front right",      () -> mod_frontRight.getState().speedMetersPerSecond)         .withPosition(1, 0);
            sb_drivetrainTab.addNumber("VEL Back left",        () -> mod_backLeft.getState().speedMetersPerSecond)           .withPosition(2, 0);
            sb_drivetrainTab.addNumber("VEL Back right",       () -> mod_backRight.getState().speedMetersPerSecond)          .withPosition(3, 0);
            sb_drivetrainTab.addNumber("DEG Front left",       () -> Math.toDegrees(mod_frontLeft.getTurnEncoderPosition())) .withPosition(0, 1);
            sb_drivetrainTab.addNumber("DEG Front right",      () -> Math.toDegrees(mod_frontRight.getTurnEncoderPosition())).withPosition(1, 1);
            sb_drivetrainTab.addNumber("DEG Back left",        () -> Math.toDegrees(mod_backLeft.getTurnEncoderPosition()))  .withPosition(2, 1);
            sb_drivetrainTab.addNumber("DEG Back right",       () -> Math.toDegrees(mod_backRight.getTurnEncoderPosition())) .withPosition(3, 1);
            sb_drivetrainTab.addNumber("ABS Front left",       () -> mod_frontLeft.getAbsoluteTurnEncoderPositionDegrees())  .withPosition(0, 2);
            sb_drivetrainTab.addNumber("ABS Front right",      () -> mod_frontRight.getAbsoluteTurnEncoderPositionDegrees()) .withPosition(1, 2);
            sb_drivetrainTab.addNumber("ABS Back left",        () -> mod_backLeft.getAbsoluteTurnEncoderPositionDegrees())   .withPosition(2, 2);
            sb_drivetrainTab.addNumber("ABS Back right",       () -> mod_backRight.getAbsoluteTurnEncoderPositionDegrees())  .withPosition(3, 2);
            sb_drivetrainTab.addNumber("POS Front left",       () -> mod_frontLeft.getPosition().distanceMeters)             .withPosition(0, 3);
            sb_drivetrainTab.addNumber("POS Front right",      () -> mod_frontRight.getPosition().distanceMeters)            .withPosition(1, 3);
            sb_drivetrainTab.addNumber("POS Back left",        () -> mod_backLeft.getPosition().distanceMeters)              .withPosition(2, 3);
            sb_drivetrainTab.addNumber("POS Back right",       () -> mod_backRight.getPosition().distanceMeters)             .withPosition(3, 3);
            sb_drivetrainTab.addNumber("GYRO Heading",         this::getHeading)                                             .withPosition(5, 0);
        }
        
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double getHeading() {
        double heading = getRotation2d().getDegrees() % 360;
        if (heading < 0) heading += 360;
        return heading;
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {mod_frontLeft.getPosition(), mod_frontRight.getPosition(), mod_backLeft.getPosition(), mod_backRight.getPosition()};
    }

    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    /**
     * Tele-op drive method
     * 
     * @param xSpeed forward/backward axis, relative to field
     * @param ySpeed left/right axis, relative to field
     * @param targetAngle target heading
     * @param rotation LT RT rotation (manual)
     */
    public void drive(double xSpeed, double ySpeed, double targetAngle, double manualRotation) {

        double currentAngle = Math.toRadians(getHeading());
        double rotation = manualRotation; // prioritize manual rotation
        headingControllerSetpoint = (targetAngle == -1) ? headingControllerSetpoint : targetAngle; // set new setpoint if available, or use old setpoint

        if (rotation == 0) {
            if (headingControllerSetpoint == -1) rotation = 0; // no target angle
            else {
                m_headingController.setSetpoint(headingControllerSetpoint);
                // if not at target angle, move to target angle
                if (Math.abs(headingControllerSetpoint - currentAngle) > m_headingController.getPositionTolerance()) // at setpoint, not using .atSetpoint() because that is not updated unless .calculate() is called
                    rotation = -m_headingController.calculate(currentAngle);
                else
                    headingControllerSetpoint = -1;
            }
        }

        // Get swerve module desired states
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, m_gyro.getRotation2d()));
        
        // Limit speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kDrive.kMaxDriveVelocity);

        // Set swerve module desired states
        setModulesStates(swerveModuleStates);

    }

    /**
     * Set swerve module states.
     * @param states array of 4 SwerveModuleStates
     */
    public void setModulesStates(SwerveModuleState[] states) {
        mod_frontLeft.setDesiredState(states[0]);
        mod_frontRight.setDesiredState(states[1]);
        mod_backLeft.setDesiredState(states[2]);
        mod_backRight.setDesiredState(states[3]);
    }

    public void resetAllEncoders() {
        mod_frontLeft.resetEncoders();
        mod_frontRight.resetEncoders();
        mod_backLeft.resetEncoders();
        mod_backRight.resetEncoders();
    }

    /**
     * Set to brake mode, or coast mode.
     * @param brakeMode whether to set brake mode, or coast mode
     */
    public void setBrakeMode(boolean brakeMode) {
        mod_frontLeft.setBrakeMode(brakeMode);
        mod_frontRight.setBrakeMode(brakeMode);
        mod_backLeft.setBrakeMode(brakeMode);
        mod_backRight.setBrakeMode(brakeMode);
    }

    public boolean getBrakeMode() {
        try {
            return mod_frontLeft.getBrakeMode();
        } catch (Exception e) {
            e.printStackTrace();
            return mod_frontRight.getBrakeMode();
        }
    }

    /**
     * Stop all motors.
     */
    public void stopMotors() {
        mod_frontLeft.stopMotors();
        mod_frontRight.stopMotors();
        mod_backLeft.stopMotors();
        mod_backRight.stopMotors();
    }

    public void enableRampRate(boolean enable) {
        mod_frontLeft.enableRampRate(enable);
        mod_frontRight.enableRampRate(enable);
        mod_backLeft.enableRampRate(enable);
        mod_backRight.enableRampRate(enable);
    }

    public boolean isRampRateEnabled() {
        return mod_frontLeft.isRampRateEnabled()
            || mod_frontRight.isRampRateEnabled()
            || mod_backLeft.isRampRateEnabled()
            || mod_backRight.isRampRateEnabled();
    }

    public void isTank(boolean isTank) {
        this.isTank = isTank;

        mod_frontLeft.getDriveMot().follow(mod_backLeft.getDriveMot());
        mod_frontRight.getDriveMot().follow(mod_backRight.getDriveMot());

        //Might need to invert idk

        m_diffDrive = new DifferentialDrive(mod_backLeft.getDriveMot(), mod_backRight.getDriveMot());
        m_diffDrive.setSafetyEnabled(false);
    }

    public boolean isTank() {
        return isTank;
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_diffDrive.arcadeDrive(xSpeed * kFallBack.maxForwardSpeed, zRotation * kFallBack.maxTurningSpeed);
    }

    public SwerveModule[] getModules() {
        SwerveModule[] modules = new SwerveModule[4];

        modules[0] = mod_frontLeft;
        modules[1] = mod_frontRight;
        modules[2] = mod_backLeft;
        modules[3] = mod_backRight;

        return modules;
    }

    @Override
    public void periodic() {
        updateOdometry();

        m_field.setRobotPose(m_odometry.getPoseMeters());
        SmartDashboard.putData(m_field);
    }
    
}
