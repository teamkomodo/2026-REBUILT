package frc.robot.util;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

import static frc.robot.Constants.*;


public class NeoSwerveModule implements SwerveModule{

    private final DoublePublisher normalizedVelocityError;
    private final DoublePublisher rotationErrorPublisher;
    private final DoublePublisher dutyCyclePublisher;
    private final DoublePublisher velocityPublisher;

    private final DoubleEntry drivekPEntry;
    private final DoubleEntry drivekIEntry;
    private final DoubleEntry drivekDEntry;
    private final DoubleEntry drivekSEntry;
    private final DoubleEntry drivekVEntry;
    private final DoubleEntry drivekAEntry;

    private final DoubleEntry steerkPEntry;
    private final DoubleEntry steerkIEntry;
    private final DoubleEntry steerkDEntry;

    private final DoubleEntry driveVelocityEntry;
    private final DoubleEntry drivePositionEntry;
    private final DoubleEntry driveVoltageEntry;

    private final DoubleEntry steerVelocityEntry;
    private final DoubleEntry steerPositionEntry;
    private final DoubleEntry steerVoltageEntry;

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMax steerMotor;
    private final SparkMaxConfig steerConfig;

    private final CANcoder steerAbsoluteEncoder;

    private SwerveModuleState desiredState;

    private final RelativeEncoder driveRelativeEncoder;
    private final RelativeEncoder steerRelativeEncoder;

    private final PIDController driveController;

    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driverController;
        
    private SimpleMotorFeedforward driveFeedforward; // Gains from SysId Analysis
    
    private double steeringOffset;
    // private double relativeSteerAdjustmentFactor = 0.1;
   // public Rotation2d angle = Rotation2d.kZero;

    public NeoSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId,double steerOffset, PIDGains steerPIDGains, PIDGains drivePIDGains, FFGains driveFFGains, NetworkTable moduleNT) {
        this.driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        this.steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));

        driveController = new PIDController(drivePIDGains.p, drivePIDGains.i, drivePIDGains.d);
        driveFeedforward = new SimpleMotorFeedforward(driveFFGains.kS, driveFFGains.kV, driveFFGains.kA);

        driveRelativeEncoder = driveMotor.getEncoder();

        steerConfig = new SparkMaxConfig();
        driveConfig = new SparkMaxConfig();

        steerAbsoluteEncoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withMagnetOffset(0.0/*steerOffset / (2 * Math.PI*)*/)
            .withAbsoluteSensorDiscontinuityPoint(getAbsoluteSensorDiscontinuity())); //correct replacement
        
        steerRelativeEncoder = steerMotor.getEncoder();
            
        //steerController = steerMotor.getPIDController();
        steerController = steerMotor.getClosedLoopController();
        driverController = driveMotor.getClosedLoopController();

        steeringOffset = steerOffset;

        configureMotors(steerPIDGains);
     
        // Telemetry
        normalizedVelocityError = moduleNT.getDoubleTopic("normvelocityerror").publish();
        rotationErrorPublisher = moduleNT.getDoubleTopic("rotationerror").publish();
        dutyCyclePublisher = moduleNT.getDoubleTopic("dutycycle").publish();
        velocityPublisher = moduleNT.getDoubleTopic("velocity").publish();
        
        drivekPEntry = moduleNT.getDoubleTopic("tuning/drivekP").getEntry(drivePIDGains.p);
        drivekIEntry = moduleNT.getDoubleTopic("tuning/drivekI").getEntry(drivePIDGains.i);
        drivekDEntry = moduleNT.getDoubleTopic("tuning/drivekD").getEntry(drivePIDGains.d);
        drivekSEntry = moduleNT.getDoubleTopic("tuning/drivekS").getEntry(driveFFGains.kS);
        drivekVEntry = moduleNT.getDoubleTopic("tuning/drivekV").getEntry(driveFFGains.kV);
        drivekAEntry = moduleNT.getDoubleTopic("tuning/drivekA").getEntry(driveFFGains.kA);

        steerkPEntry = moduleNT.getDoubleTopic("tuning/steerkP").getEntry(steerPIDGains.p);
        steerkIEntry = moduleNT.getDoubleTopic("tuning/steerkI").getEntry(steerPIDGains.i);
        steerkDEntry = moduleNT.getDoubleTopic("tuning/steerkD").getEntry(steerPIDGains.d);

        driveVelocityEntry = moduleNT.getDoubleTopic("drive/velocity").getEntry(driveRelativeEncoder.getVelocity());
        drivePositionEntry = moduleNT.getDoubleTopic("drive/position").getEntry(driveRelativeEncoder.getPosition());
        driveVoltageEntry = moduleNT.getDoubleTopic("drive/voltage").getEntry(driveMotor.getBusVoltage());

        steerVelocityEntry = moduleNT.getDoubleTopic("steer/velocity").getEntry(steerRelativeEncoder.getVelocity());
        steerPositionEntry = moduleNT.getDoubleTopic("steer/position").getEntry(steerRelativeEncoder.getPosition());
        steerVoltageEntry = moduleNT.getDoubleTopic("steer/voltage").getEntry(steerMotor.getBusVoltage());

        drivekPEntry.set(drivePIDGains.p);
        drivekIEntry.set(drivePIDGains.i);
        drivekDEntry.set(drivePIDGains.d);
        drivekSEntry.set(driveFFGains.kS);
        drivekVEntry.set(driveFFGains.kV);
        drivekAEntry.set(driveFFGains.kA);

        steerkPEntry.set(steerPIDGains.p);
        steerkIEntry.set(steerPIDGains.i);
        steerkDEntry.set(steerPIDGains.d);

        driveVelocityEntry.set(getDriveVelocity());
        drivePositionEntry.set(getDrivePosition());
        driveVoltageEntry.set(getDriveVoltage());

        steerVelocityEntry.set(getSteerVelocity());
        steerPositionEntry.set(getSteerPosition());
        steerVoltageEntry.set(getSteerVoltage());
    }

    @SuppressWarnings("removal")
    private void configureMotors(PIDGains steerGains) {

        //Drive Motors
        driveConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        double wheelPositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION; // motor rotations -> wheel travel in meters

        driveConfig.encoder
        .positionConversionFactor(wheelPositionConversionFactor)
        .velocityConversionFactor(wheelPositionConversionFactor / 60); // motor RPM -> wheel speed in m/s

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Steer Motor
        steerConfig.inverted(true)
        .idleMode(IdleMode.kBrake);

        steerConfig.encoder
        .positionConversionFactor(2 * Math.PI * STEER_REDUCTION)
        .velocityConversionFactor(2 * Math.PI * STEER_REDUCTION / 60);

        steerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(Math.PI)
        .positionWrappingMinInput(-Math.PI)
        .pid(steerGains.p, steerGains.i,steerGains.d);

        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
       steerRelativeEncoder.setPosition(getAbsoluteModuleRotation().getRadians());
    }

    public void updateSysIDValues() {
        driveVelocityEntry.set(getDriveVelocity());
        drivePositionEntry.set(getDrivePosition());
        driveVoltageEntry.set(getDriveVoltage());

        steerVelocityEntry.set(getSteerVelocity());
        steerPositionEntry.set(getSteerPosition());
        steerVoltageEntry.set(getSteerVoltage());
    }

    public void updateTelemetry(){
        normalizedVelocityError.set((desiredState.speedMetersPerSecond - getDriveVelocity()) * Math.signum(desiredState.speedMetersPerSecond));
        rotationErrorPublisher.set(MathUtil.angleModulus(desiredState.angle.getRadians() - getModuleRotation().getRadians()));
        dutyCyclePublisher.set(driveMotor.get());
        velocityPublisher.set(getDriveVelocity(), RobotController.getFPGATime() - 200000);

        if(!TUNING_MODE)
            return;

        double newDrivekP = drivekPEntry.get();
        if(newDrivekP != driveController.getP()) driveController.setP(newDrivekP);

        double newDrivekI = drivekIEntry.get();
        if(newDrivekI != driveController.getI()) driveController.setI(newDrivekI);

        double newDrivekD = drivekDEntry.get();
        if(newDrivekD != driveController.getD()) driveController.setD(newDrivekD);

        double newDrivekS = drivekSEntry.get();
        double newDrivekV = drivekVEntry.get();
        double newDrivekA = drivekAEntry.get();

        if(newDrivekS != driveFeedforward.getKs() || newDrivekV != driveFeedforward.getKv() || newDrivekA != driveFeedforward.getKa())
            driveFeedforward = new SimpleMotorFeedforward(newDrivekS, newDrivekV, newDrivekA);
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveRelativeEncoder.getVelocity(), getModuleRotation());
    }

    public SwerveModuleState getDesiredState(){
        return desiredState;
       // return null;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveRelativeEncoder.getPosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState inputSwerveState) {
       inputSwerveState.optimize(getModuleRotation());
       this.desiredState = inputSwerveState;
    }

    @Override
    public void periodic() {
        final double driveOutput = driveController.calculate(driveRelativeEncoder.getVelocity(), desiredState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        //System.out.println(driveFeedforward);
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        steerController.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);
        updateSysIDValues();
    }

    public Rotation2d getModuleRotation() {
        return new Rotation2d(steerRelativeEncoder.getPosition());
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI + steeringOffset);
    }
    
    private double getDriveVelocity(){
        return driveRelativeEncoder.getVelocity();
    }

    private double getSteerVelocity(){
        return steerRelativeEncoder.getVelocity();
    }

    private double getDriveVoltage() {
        return driveMotor.getOutputCurrent() * driveMotor.getBusVoltage(); 
    }

    private double getSteerVoltage() {
        return steerMotor.getAppliedOutput() * steerMotor.getBusVoltage(); 
    }

    public double getDrivePosition() {
        return driveRelativeEncoder.getPosition();
    }

    public double getSteerPosition() {
        return steerRelativeEncoder.getPosition();
    }

    private Angle getAbsoluteSensorDiscontinuity(){
        return new MagnetSensorConfigs().getAbsoluteSensorDiscontinuityPointMeasure();
    }

    @Override
    public void runForward(double voltage) {
        driveMotor.setVoltage(voltage);
        steerController.setSetpoint(0, ControlType.kPosition);
    }

    @Override
    public void runRotation(double voltage) {
        driveMotor.set(0);
        steerMotor.setVoltage(voltage);
    }

}