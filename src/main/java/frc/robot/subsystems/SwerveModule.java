package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.kDrive;

/**
 * SDS MK4i Swerve Module
 * NEO Pinions, L2
 * Product page: https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777303153
 */
public class SwerveModule {

    // Motors
    private final CANSparkMax mot_drive;
    private final CANSparkMax mot_turn;

    // Encoders
    private final WPI_CANCoder enc_drive;
    private final WPI_CANCoder enc_turn;
    private final CANCoderConfiguration enc_driveConfig;
    private final CANCoderConfiguration enc_turnConfig;

    // PID Controllers
    private final PIDController m_drivePIDController;
    private final ProfiledPIDController m_turnPIDController;

    // Feedforward
    private final SimpleMotorFeedforward m_driveFF;
    private final SimpleMotorFeedforward m_turnFF;


    public SwerveModule(int driveMotorID, int turnMotorID, int driveEncoderID, int turnEncoderID) {

        // Motors
        mot_drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mot_turn = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        // Encoders
        enc_drive = new WPI_CANCoder(driveEncoderID);
        enc_turn = new WPI_CANCoder(turnEncoderID);
        enc_driveConfig = new CANCoderConfiguration();
        enc_turnConfig = new CANCoderConfiguration();
        configEncoder();

        // PID Controllers
        m_drivePIDController = new PIDController(kDrive.kDriveP, kDrive.kDriveI, kDrive.kDriveD);
        m_turnPIDController = new ProfiledPIDController(kDrive.kTurnP, kDrive.kTurnI, kDrive.kTurnD, new TrapezoidProfile.Constraints(kDrive.kMaxDriveAngularVelocity, kDrive.kMaxTurnAngularAcceleration));
        m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Feedforwards
        m_driveFF = new SimpleMotorFeedforward(kDrive.kDriveFFkS, kDrive.kDriveFFkV);
        m_turnFF = new SimpleMotorFeedforward(kDrive.kTurnFFkS, kDrive.kTurnFFkV);
    }

    private void configEncoder() {
        enc_driveConfig.sensorCoefficient = kDrive.kEncoder.kDriveSensorCoefficient;
        enc_driveConfig.unitString = kDrive.kEncoder.kUnitString;
        enc_driveConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_drive.configAllSettings(enc_driveConfig);
        
        enc_turnConfig.sensorCoefficient = kDrive.kEncoder.kDriveSensorCoefficient;
        enc_turnConfig.unitString = kDrive.kEncoder.kUnitString;
        enc_turnConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_turn.configAllSettings(enc_turnConfig);
    }

    // public SwerveModuleState getState() {
         
    // }
    
}
