package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.*;

public class YagslModule {

    private SparkMax driveMotor;
    private SparkMax steerMotor;
    private PIDController drivingController;
    private SparkClosedLoopController turningController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    private SparkMaxConfig driveConfig;
    private SparkMaxConfig steerConfig;

    private SwerveModuleState state;
    private double steerOffset;
    private SimpleMotorFeedforward drivingFeedforward;
    private CANcoder moduleCANcoder;
    private CANcoderConfigurator moduleConfigutaror;
    private MagnetSensorConfigs  magnetSensorConfiguration;

    private final DoublePublisher normalizedVelocityError;
    private final DoublePublisher rotationErrorPublisher;
    private final DoublePublisher dutyCyclePublisher;
    private final DoublePublisher velocityPublisher;
    private final DoubleEntry driveVelocityEntry;
    private final DoubleEntry drivePositionEntry;
    private final DoubleEntry driveVoltageEntry;
    private final DoubleEntry steerVelocityEntry;
    private final DoubleEntry steerPositionEntry;
    private final DoubleEntry steerVoltageEntry;
    
    public YagslModule(int driveMotorCANID, 
                       int steerMotorCANID, 
                       int cancoderCANID,
                       double steerOffset,
                       PIDGains steerGains,
                       PIDGains driveGains,
                       FFGains driveFFGains,
                       NetworkTable moduleNT){
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        
        // Get the PID Controllers
        drivingController = new PIDController(driveGains.p, driveGains.i, driveGains.d);
        drivingFeedforward = new SimpleMotorFeedforward(driveFFGains.kS, driveFFGains.kV, driveFFGains.kA);
        turningController = steerMotor.getClosedLoopController();
        
        // Get the encoders
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();
        
        // Continue configuration here..
        
        // CANcoder Configuration
        moduleCANcoder = new CANcoder(cancoderCANID);
        moduleConfigutaror = moduleCANcoder.getConfigurator();
        magnetSensorConfiguration = new MagnetSensorConfigs();

        steerConfig = new SparkMaxConfig();
        driveConfig = new SparkMaxConfig();

        this.steerOffset = steerOffset;

        configureModule(steerGains);
        
        normalizedVelocityError = moduleNT.getDoubleTopic("normvelocityerror").publish();
        rotationErrorPublisher = moduleNT.getDoubleTopic("rotationerror").publish();
        dutyCyclePublisher = moduleNT.getDoubleTopic("dutycycle").publish();
        velocityPublisher = moduleNT.getDoubleTopic("velocity").publish();

        driveVelocityEntry = moduleNT.getDoubleTopic("drive/velocity").getEntry(driveEncoder.getVelocity());
        drivePositionEntry = moduleNT.getDoubleTopic("drive/position").getEntry(driveEncoder.getPosition());
        driveVoltageEntry = moduleNT.getDoubleTopic("drive/voltage").getEntry(driveMotor.getBusVoltage());

        steerVelocityEntry = moduleNT.getDoubleTopic("steer/velocity").getEntry(steerEncoder.getVelocity());
        steerPositionEntry = moduleNT.getDoubleTopic("steer/position").getEntry(steerEncoder.getPosition());
        steerVoltageEntry = moduleNT.getDoubleTopic("steer/voltage").getEntry(steerMotor.getBusVoltage());

        driveVelocityEntry.set(getDriveVelocity());
        drivePositionEntry.set(getDrivePosition());
        driveVoltageEntry.set(getDriveVoltage());

        steerVelocityEntry.set(getSteerVelocity());
        steerPositionEntry.set(getSteerPosition());
        steerVoltageEntry.set(getSteerVoltage());
    }

    @SuppressWarnings("removal")
    public void configureModule(PIDGains steerGains) {
        // Reset everything to factory default
        // driveMotor.restoreFactoryDefaults(); FIXME
        // steerMotor.restoreFactoryDefaults(); FIXME

        moduleConfigutaror.apply(new CANcoderConfiguration());
        moduleConfigutaror.refresh(magnetSensorConfiguration);
        moduleConfigutaror.apply(magnetSensorConfiguration
                  .withMagnetOffset(0.0)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withAbsoluteSensorDiscontinuityPoint(getAbsoluteSensorDiscontinuity()));
                  // .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1) FIXME
        
        // Steering Motor Configuration
        steerConfig
            .inverted(true)
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve APIs.
        steerConfig.encoder
            .positionConversionFactor(STEER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(STEER_VELOCITY_CONVERSION_FACTOR);
        
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingMaxInput(Math.PI)
            .positionWrappingMinInput(-Math.PI)
            // Set the PID gains for the turning motor. Note these are example gains, and you
            // may need to tune them for your own robot!
            .pid(steerGains.p, steerGains.i, steerGains.d)
            .velocityFF(steerGains.FF); //FIXME
            

        //turningPIDController.setFeedbackDevice(steerEncoder); FIXME
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerEncoder.setPosition(getAbsoluteModuleRotation().getRadians());
        // Drive Motor Configuration
        driveConfig
            .inverted(true)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveConfig.encoder
            .positionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR); //FIXME
        
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        // driveConfig.closedLoop //FIXME
        //     .pid(driveGains.p, driveGains.i, driveGains.d) //FIXME
        //     .velocityFF(driveGains.FF); FIXME

        //drivingPIDController.setFeedbackDevice(driveEncoder); FIXME
        
        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        // driveMotor.burnFlash(); FIXME
        // steerMotor.burnFlash(); FIXME
          
        driveEncoder.setPosition(0);
    }

    public void periodic() {
        updateTelemetry();
        optimize();
    }
    
    // Get the distance in meters
    public double getDistance() {
        return driveEncoder.getPosition();
    }
    
    // Get the angle
    public Rotation2d getAngle() {
          //return Rotation2d.fromDegrees(steerEncoder.getPosition());
          return Rotation2d.fromRadians(steerEncoder.getPosition());
    }
    
    /** Set the swerve module state.
    @param state The swerve module state to set. */
    public void setState(SwerveModuleState state) {
        final double driveOutput = drivingController.calculate(driveEncoder.getVelocity(),
            state.speedMetersPerSecond);
        final double driveFeedforward = this.drivingFeedforward.calculate(state.speedMetersPerSecond);

        //driveMotor.setVoltage(driveOutput + driveFeedforward);
        driveMotor.setVoltage(MathUtil.clamp(driveOutput + driveFeedforward, -12, 12));

        double minInputAngle = getModuleRotation().getRadians() - Math.PI;
        double maxInputAngle = getModuleRotation().getRadians() + Math.PI;
        double inputAngle = MathUtil.inputModulus(state.angle.getRadians(), minInputAngle, maxInputAngle);

        //turningController.setSetpoint(state.angle.getDegrees(), ControlType.kPosition); FIXME
        turningController.setSetpoint(inputAngle, ControlType.kPosition);
    }

    public void setDesiredState(SwerveModuleState inputSwerveState) {
        state = inputSwerveState;
    }

    public void optimize() {
        state.optimize(getModuleRotation());
        setState(state);
    }

    private Angle getAbsoluteSensorDiscontinuity() {
        return new MagnetSensorConfigs().getAbsoluteSensorDiscontinuityPointMeasure();
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(moduleCANcoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI + steerOffset);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getModuleRotation());
    }

    public Rotation2d getModuleRotation() {
        return new Rotation2d(steerEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getModuleRotation());
    }

    public SwerveModuleState getDesiredState() {
        return state;
    }

    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    private double getDriveVoltage() {
        return driveMotor.getOutputCurrent() * driveMotor.getBusVoltage();
    }

    private double getSteerVoltage() {
        return steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    public void updateTelemetry() {
        normalizedVelocityError.set((state.speedMetersPerSecond - getDriveVelocity())
                * Math.signum(state.speedMetersPerSecond));
        rotationErrorPublisher
                .set(MathUtil.angleModulus(state.angle.getRadians() - getModuleRotation().getRadians()));
        dutyCyclePublisher.set(driveMotor.get());
        velocityPublisher.set(getDriveVelocity(), RobotController.getFPGATime() - 200000);
    }
}