package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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


    public SwerveModule(int driveMotorID, int turnMotorID, int driveEncoderID, int turnEncoderID,
        boolean driveMotorInverted, boolean turnMotorInverted) {

        // Motors
        mot_drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mot_turn = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        configMotors(driveMotorInverted, turnMotorInverted);

        // Encoders
        enc_drive = new WPI_CANCoder(driveEncoderID);
        enc_turn = new WPI_CANCoder(turnEncoderID);
        enc_driveConfig = new CANCoderConfiguration();
        enc_turnConfig = new CANCoderConfiguration();
        configEncoder();

        // PID Controllers
        m_drivePIDController = new PIDController(kDrive.kDriveP, kDrive.kDriveI, kDrive.kDriveD);
        m_turnPIDController = new ProfiledPIDController(kDrive.kTurnP, kDrive.kTurnI, kDrive.kTurnD,
            new TrapezoidProfile.Constraints(kDrive.kMaxDriveAngularVelocity, kDrive.kMaxTurnAngularAcceleration));
        m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Feedforwards
        m_driveFF = new SimpleMotorFeedforward(kDrive.kDriveFFkS, kDrive.kDriveFFkV);
        m_turnFF = new SimpleMotorFeedforward(kDrive.kTurnFFkS, kDrive.kTurnFFkV);
    }

    /**
     * Config the drive and turn motors.
     * 
     * - Set brake mode
     * - Set inverted (if applicable)
     * - Set current limit
     */
    private void configMotors(boolean driveMotorInverted, boolean turnMotorInverted) {
        mot_drive.restoreFactoryDefaults();
        mot_drive.setIdleMode(IdleMode.kBrake);
        mot_drive.setInverted(driveMotorInverted);
        mot_drive.setSmartCurrentLimit(kDrive.kDriveMotorCurrentLimit);

        mot_turn.restoreFactoryDefaults();
        mot_turn.setIdleMode(IdleMode.kBrake);
        mot_turn.setInverted(turnMotorInverted);
        mot_turn.setSmartCurrentLimit(kDrive.kTurnMotorCurrentLimit);
    }

    /**
     * Configure the drive and turn CANCoders.
     */
    private void configEncoder() {
        enc_driveConfig.sensorCoefficient = kDrive.kEncoder.kDriveSensorCoefficient;
        enc_driveConfig.unitString = kDrive.kEncoder.kUnitString;
        enc_driveConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_drive.configAllSettings(enc_driveConfig);
        
        enc_turnConfig.sensorCoefficient = kDrive.kEncoder.kDriveSensorCoefficient;
        enc_turnConfig.unitString = kDrive.kEncoder.kUnitString;
        enc_turnConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_turn.configAllSettings(enc_turnConfig);

        resetEncoders();
    }

    public void stopMotors() {
        mot_drive.set(0);
        mot_turn.set(0);
    }

    /**
     * Set drive and turning encoder positions to zero.
     */
    public void resetEncoders() {
        enc_drive.setPosition(0);
        enc_turn.setPosition(0);
    }

    /**
     * @return current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(enc_drive.getVelocity(), new Rotation2d(enc_turn.getPosition()));
    }

    /**
     * @return current position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(enc_drive.getPosition(), new Rotation2d(enc_turn.getPosition()));
    }

    /**
     * Set desired state of the module.
     * 
     * @param desiredState desired state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // If no velocity, don't set new module state.
        // This will prevent the wheels from turning back to straight
        // each time after moving.
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stopMotors();
            return;
        }

        // Optimize reference state
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(enc_turn.getPosition()));

        // Calculate drive output
        final double driveOutput = m_drivePIDController.calculate(enc_drive.getPosition(), optimizedState.speedMetersPerSecond);
        final double driveFF = m_driveFF.calculate(optimizedState.speedMetersPerSecond);

        // Calculate turn output
        final double turnOutput = m_turnPIDController.calculate(enc_turn.getPosition(), optimizedState.angle.getRadians());
        final double turnFF = m_turnFF.calculate(m_turnPIDController.getSetpoint().velocity);

        // Set motor voltage
        mot_drive.setVoltage(driveOutput + driveFF);
        mot_turn.setVoltage(turnOutput + turnFF);
    }
    
}
