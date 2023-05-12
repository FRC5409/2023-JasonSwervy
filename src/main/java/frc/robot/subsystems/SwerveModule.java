package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.kCANCoder;
import frc.robot.Constants.kDrive.kRelativeEncoder;

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
    private final RelativeEncoder enc_drive;
    private final RelativeEncoder enc_turn;
    private final WPI_CANCoder enc_cancoder;
    private final CANCoderConfiguration m_cancoderConfiguration;

    // PID Controllers
    private final PIDController m_drivePIDController;
    private final ProfiledPIDController m_turnPIDController;

    // Feedforward
    private final SimpleMotorFeedforward m_driveFF;
    private final SimpleMotorFeedforward m_turnFF;


    public SwerveModule(int driveMotorID, int turnMotorID, int driveEncoderID, int turnEncoderID,
        int cancoderID, boolean driveMotorInverted, boolean turnMotorInverted) {

        // Motors
        mot_drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mot_turn = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        configMotors(driveMotorInverted, turnMotorInverted);

        // Encoders
        enc_drive = mot_drive.getEncoder();
        enc_turn = mot_turn.getEncoder();
        enc_cancoder = new WPI_CANCoder(cancoderID);
        m_cancoderConfiguration = new CANCoderConfiguration();
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
        enc_drive.setVelocityConversionFactor(kRelativeEncoder.kDriveSensorCoefficient * 60);
        enc_drive.setPositionConversionFactor(kRelativeEncoder.kDriveSensorCoefficient * 60);
        enc_turn.setVelocityConversionFactor(kRelativeEncoder.kTurnSensorCoefficient * 60);
        enc_turn.setPositionConversionFactor(kRelativeEncoder.kTurnSensorCoefficient * 60);
        
        m_cancoderConfiguration.magnetOffsetDegrees = kCANCoder.kAbsoluteEncoderOffset;
        m_cancoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        enc_cancoder.configAllSettings(m_cancoderConfiguration);

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
