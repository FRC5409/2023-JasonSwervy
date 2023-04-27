package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
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
    

    public Drivetrain() {

        // Swerve modules
        mod_frontLeft = new SwerveModule(kCANID.kDriveMotor1, kCANID.kTurnMotor1, kCANID.kDriveEncoder1, kCANID.kTurnEncoder1);
        mod_frontRight = new SwerveModule(kCANID.kDriveMotor2, kCANID.kTurnMotor2, kCANID.kDriveEncoder2, kCANID.kTurnEncoder2);
        mod_backLeft = new SwerveModule(kCANID.kDriveMotor3, kCANID.kTurnMotor3, kCANID.kDriveEncoder3, kCANID.kTurnEncoder3);
        mod_backRight = new SwerveModule(kCANID.kDriveMotor4, kCANID.kTurnMotor4, kCANID.kDriveEncoder4, kCANID.kTurnEncoder4);

        // Swerve kinematic points
        m_frontLeftLoc = new Translation2d(kRobot.length / 2, kRobot.width / 2);
        m_frontRightLoc = new Translation2d(kRobot.length / 2, -kRobot.width / 2);
        m_backLeftLoc = new Translation2d(-kRobot.length / 2, kRobot.width / 2);
        m_backRightLoc = new Translation2d(-kRobot.length / 2, -kRobot.width / 2);

        // Sensors and location
        m_gyro = new WPI_Pigeon2(kCANID.kGyro);
        m_gyro.reset();
        m_kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] {mod_frontLeft.getPosition(), mod_frontRight.getPosition(),mod_backLeft.getPosition(), mod_backRight.getPosition()});
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
}
