package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.Location;
import frc.robot.Constants.kDrive.kRelativeEncoder;

/**
 * SDS MK4i Swerve Module
 * NEO Pinions, L2
 * Product page: https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777303153
 */
public class SwerveModule extends SubsystemBase {

    // Motors
    private final CANSparkMax mot_drive;
    private final CANSparkMax mot_turn;

    // Encoders
    private final RelativeEncoder enc_drive;
    private final RelativeEncoder enc_turn;
    private final WPI_CANCoder enc_cancoder;
    private final CANCoderConfiguration m_cancoderConfiguration;

    // PID Controllers
    private final SparkMaxPIDController m_drivePIDController;
    private final SimpleMotorFeedforward m_driveFF;
    private final SparkMaxPIDController m_turnPIDController;

    // Location
    private final Location m_location;

    private boolean isStuck = false;

    public SwerveModule(int driveMotorID, int turnMotorID,
        int cancoderID, double cancoderAbsoluteOffset,
        boolean driveMotorInverted, boolean turnMotorInverted, Location location) {

        // Motors
        mot_drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mot_turn = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        // Encoders
        enc_drive = mot_drive.getEncoder();
        enc_turn = mot_turn.getEncoder();
        enc_cancoder = new WPI_CANCoder(cancoderID);
        m_cancoderConfiguration = new CANCoderConfiguration();

        // PID Controllers
        m_drivePIDController = mot_drive.getPIDController();
        m_driveFF = new SimpleMotorFeedforward(kDrive.kDriveKs, kDrive.kDriveKv);
        m_turnPIDController = mot_turn.getPIDController();

        // Location
        m_location = location;

        configMotorsAndEncoders(driveMotorInverted, turnMotorInverted, -cancoderAbsoluteOffset);

        resetEncoders();
    }

    /**
     * Config the drive and turn motors.
     * 
     * - Set brake mode
     * - Set inverted (if applicable)
     * - Set current limit
     */
    private void configMotorsAndEncoders(boolean driveMotorInverted, boolean turnMotorInverted, double cancoderAbsoluteOffset) {
        mot_drive.restoreFactoryDefaults();
        mot_drive.setInverted(driveMotorInverted);
        mot_drive.setSmartCurrentLimit(kDrive.kDriveMotorCurrentLimit);
        enableRampRate(true);
        m_drivePIDController.setP(kDrive.kDriveP);
        m_drivePIDController.setI(kDrive.kDriveI);
        m_drivePIDController.setD(kDrive.kDriveD);

        mot_turn.restoreFactoryDefaults();
        mot_turn.setInverted(turnMotorInverted);
        mot_turn.setSmartCurrentLimit(kDrive.kTurnMotorCurrentLimit);
        m_turnPIDController.setP(kDrive.kTurnP);
        m_turnPIDController.setI(kDrive.kTurnI);
        m_turnPIDController.setD(kDrive.kTurnD);
        m_turnPIDController.setFF(kDrive.kTurnFF);
        m_turnPIDController.setSmartMotionMaxAccel(kDrive.kMaxTurnAngularAcceleration, 0);
        m_turnPIDController.setPositionPIDWrappingEnabled(true);
        m_turnPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
        m_turnPIDController.setPositionPIDWrappingMinInput(0);

        // Set to coast mode on startup. Will set to brake mode on enable.
        setBrakeMode(false);

        configEncoder(cancoderAbsoluteOffset);

        mot_drive.burnFlash();
        mot_turn.burnFlash();
    }

    /**
     * Configure the drive and turn CANCoders.
     */
    private void configEncoder(double cancoderAbsoluteOffset) {
        enc_drive.setVelocityConversionFactor(kRelativeEncoder.kDriveSensorCoefficient / 60);
        enc_drive.setPositionConversionFactor(kRelativeEncoder.kDriveSensorCoefficient);
        enc_turn.setVelocityConversionFactor(kRelativeEncoder.kTurnSensorCoefficient / 60);
        enc_turn.setPositionConversionFactor(kRelativeEncoder.kTurnSensorCoefficient);
        
        m_cancoderConfiguration.magnetOffsetDegrees = cancoderAbsoluteOffset;
        m_cancoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        enc_cancoder.configAllSettings(m_cancoderConfiguration);
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
        enc_turn.setPosition(getAbsoluteTurnEncoderPositionRadians());
    }

    /**
     * Return the absolute turn encoder position in degrees [0-360].
     * 
     * @return absolute turn encoder position
     */
    public double getAbsoluteTurnEncoderPositionDegrees() {
        return enc_cancoder.getAbsolutePosition();
    }

    /**
     * Return the absolute turn encoder position in radians.
     * 
     * @return absolute turn encoder position
     */
    public double getAbsoluteTurnEncoderPositionRadians() {
        return Math.toRadians(getAbsoluteTurnEncoderPositionDegrees());
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
        SwerveModuleState optimizedState = optimize(desiredState, new Rotation2d(enc_turn.getPosition()));

        // SmartDashboard.putNumber(m_location + " Vel", optimizedState.speedMetersPerSecond);
        // SmartDashboard.putNumber(m_location + " Deg", optimizedState.angle.getDegrees());

        // Drive output
        m_drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0, m_driveFF.calculate(optimizedState.speedMetersPerSecond));
        m_turnPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    }

     /**
    * Minimize the change in heading the desired swerve module state would require by potentially
    * reversing the direction the wheel spins. If this is used with the PIDController class's
    * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
    *
    * @param desiredState The desired state.
    * @param currentAngle The current module angle.
    * @return Optimized swerve module state.
    */
    public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 120.0) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
      } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    /**
     * Sets the turn motor to a specified degree
     * @param degrees
     */
    public void setTurnDegrees(double degrees) {
        setTurnRadians(Math.toRadians(degrees));
    }

    /**
     * Sets the turn motor to a specified radian
     * @param radians
     */
    public void setTurnRadians(double radians) {
        m_turnPIDController.setReference(radians, ControlType.kPosition);
    }

    public double getTurnEncoderPosition() {
        return enc_turn.getPosition();
    }

    /**
     * Set to brake mode, or coast mode.
     * @param brakeMode whether to set brake mode, or coast mode
     */
    public void setBrakeMode(boolean brakeMode) {
        if (brakeMode) {
            mot_drive.setIdleMode(IdleMode.kBrake);
            mot_turn.setIdleMode(IdleMode.kBrake);
        } else {
            mot_drive.setIdleMode(IdleMode.kCoast);
            mot_turn.setIdleMode(IdleMode.kCoast);
        }
    }

    public boolean getBrakeMode() {
        if (mot_drive.getIdleMode() == IdleMode.kBrake) return true;
        else                                            return false;
    }

    public CANSparkMax getDriveMot() {
        return mot_drive;
    }

    public void enableRampRate(boolean enable) {
        mot_drive.setClosedLoopRampRate(enable ? kDrive.kDriveRampRate : 0);
    }

    public boolean isRampRateEnabled() {
        return mot_drive.getClosedLoopRampRate() > 0;
    }

    public void isTurnStuck(boolean stuck) {
        isStuck = stuck;
    }

    public boolean isStuck() {
        return isStuck;
    }

    @Override
    public void periodic() {

    }
    
}
