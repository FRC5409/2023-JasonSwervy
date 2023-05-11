package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kDrive;
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

    // Shuffleboard
    private boolean debugMode = true;
    private ShuffleboardTab sb_drivetrainTab;
    

    public Drivetrain() {

        // Swerve modules
        mod_frontLeft = new SwerveModule(kCANID.kDriveMotor1, kCANID.kTurnMotor1, kCANID.kDriveEncoder1, kCANID.kTurnEncoder1, true, true);
        mod_frontRight = new SwerveModule(kCANID.kDriveMotor2, kCANID.kTurnMotor2, kCANID.kDriveEncoder2, kCANID.kTurnEncoder2, false, false);
        mod_backLeft = new SwerveModule(kCANID.kDriveMotor3, kCANID.kTurnMotor3, kCANID.kDriveEncoder3, kCANID.kTurnEncoder3, true, true);
        mod_backRight = new SwerveModule(kCANID.kDriveMotor4, kCANID.kTurnMotor4, kCANID.kDriveEncoder4, kCANID.kTurnEncoder4, false, false);

        // Swerve kinematic points
        m_frontLeftLoc = new Translation2d(kRobot.length / 2, kRobot.width / 2);
        m_frontRightLoc = new Translation2d(kRobot.length / 2, -kRobot.width / 2);
        m_backLeftLoc = new Translation2d(-kRobot.length / 2, kRobot.width / 2);
        m_backRightLoc = new Translation2d(-kRobot.length / 2, -kRobot.width / 2);

        // Sensors and location
        m_gyro = new WPI_Pigeon2(kCANID.kGyro);
        zeroHeading();
        m_kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] {mod_frontLeft.getPosition(), mod_frontRight.getPosition(),mod_backLeft.getPosition(), mod_backRight.getPosition()});
        m_field = new Field2d();

        // Shuffleboard
        if (debugMode) {
            sb_drivetrainTab = Shuffleboard.getTab("Drivetrain");
            sb_drivetrainTab.addNumber("VEL Front left",    () -> mod_frontLeft.getState().speedMetersPerSecond)  .withPosition(1, 1);
            sb_drivetrainTab.addNumber("VEL Front right",   () -> mod_frontRight.getState().speedMetersPerSecond) .withPosition(2, 1);
            sb_drivetrainTab.addNumber("VEL Back left",     () -> mod_backLeft.getState().speedMetersPerSecond)   .withPosition(3, 1);
            sb_drivetrainTab.addNumber("VEL Back right",    () -> mod_backRight.getState().speedMetersPerSecond)  .withPosition(4, 1);
            sb_drivetrainTab.addNumber("ANGLE Front left",  () -> mod_frontLeft.getState().angle.getDegrees())    .withPosition(1, 1);
            sb_drivetrainTab.addNumber("ANGLE Front right", () -> mod_frontRight.getState().angle.getDegrees())   .withPosition(2, 1);
            sb_drivetrainTab.addNumber("ANGLE Back left",   () -> mod_backLeft.getState().angle.getDegrees())     .withPosition(3, 1);
            sb_drivetrainTab.addNumber("ANGLE Back right",  () -> mod_backRight.getState().angle.getDegrees())    .withPosition(4, 1);
        }
        
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(),
            new SwerveModulePosition[] {mod_frontLeft.getPosition(), mod_frontRight.getPosition(),mod_backLeft.getPosition(), mod_backRight.getPosition()});
    }

    /**
     * Tele-op drive method
     * 
     * @param xSpeed forward/backward axis, relative to field
     * @param ySpeed left/right axis, relative to field
     * @param rotation angular rate
     */
    public void drive(double xSpeed, double ySpeed, double rotation) {

        // Get swerve module desired states
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, m_gyro.getRotation2d()));
        
        // Limit speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kDrive.kMaxDriveVelocity);

        // Set swerve module desired states
        mod_frontLeft.setDesiredState(swerveModuleStates[0]);
        mod_frontRight.setDesiredState(swerveModuleStates[1]);
        mod_backLeft.setDesiredState(swerveModuleStates[2]);
        mod_backRight.setDesiredState(swerveModuleStates[3]);

    }

    public void stopMotors() {
        mod_frontLeft.stopMotors();
        mod_frontRight.stopMotors();
        mod_backLeft.stopMotors();
        mod_backRight.stopMotors();
    }

    @Override
    public void periodic() {
        updateOdometry();

        m_field.setRobotPose(m_odometry.getPoseMeters());
        SmartDashboard.putData(m_field);
    }
    
}
