package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

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
    private CANcoder absoluteEncoder;
    private PIDController drivingController;
    private SparkClosedLoopController turningController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    private SparkMaxConfig driveConfig;
    private SparkMaxConfig steerConfig;

    private double steerOffset;
    private SimpleMotorFeedforward drivingFeedforward;
    private CANcoder moduleCANcoder;
    private CANcoderConfigurator moduleConfigutaror;
    private MagnetSensorConfigs  magnetSensorConfiguration;
    
    public YagslModule(int driveMotorCANID, 
                       int steerMotorCANID, 
                       int cancoderCANID,
                       double steerOffset,
                       PIDGains steerGains,
                       PIDGains driveGains,
                       FFGains driveFFGains){
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderCANID);
        
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
    }

    @SuppressWarnings("removal")
    public void configureModule(PIDGains steerGains) {
        // Reset everything to factory default
        // driveMotor.restoreFactoryDefaults(); FIXME
        // steerMotor.restoreFactoryDefaults(); FIXME
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

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
            .idleMode(IdleMode.kBrake);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve APIs.
        steerConfig.encoder
            .positionConversionFactor(2 * Math.PI * STEER_REDUCTION)
            .velocityConversionFactor(2 * Math.PI * STEER_REDUCTION / 60);
        
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
            .idleMode(IdleMode.kBrake);
        
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveConfig.encoder
            .positionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR); //FIXME
        
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
    
    // Get the distance in meters
    public double getDistance() {
        return driveEncoder.getPosition();
    }
    
    // Get the angle
    public Rotation2d getAngle() {
          return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
    
    /** Set the swerve module state.
    @param state The swerve module state to set. */
    public void setState(SwerveModuleState state) {
        final double driveOutput = drivingController.calculate(driveEncoder.getVelocity(),
                state.speedMetersPerSecond);
        final double driveFeedforward = this.drivingFeedforward.calculate(state.speedMetersPerSecond);
          turningController.setSetpoint(state.angle.getDegrees(), ControlType.kPosition);
          driveMotor.setVoltage(driveOutput + driveFeedforward);
    }

    public void setDesiredState(SwerveModuleState inputSwerveState) {
        inputSwerveState.optimize(getModuleRotation());
        setState(inputSwerveState);
        //this.state = inputSwerveState;
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
}