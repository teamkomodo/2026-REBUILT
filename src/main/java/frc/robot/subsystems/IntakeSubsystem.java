package frc.robot.subsystems;

import static frc.robot.Constants.BRUSHLESS;
import static frc.robot.Constants.HINGE_EJECT_POSITION;
import static frc.robot.Constants.HINGE_FEED_POSITION;
import static frc.robot.Constants.HINGE_INTAKE_POSITION;
import static frc.robot.Constants.HINGE_MOTOR_LEFT_ID;
import static frc.robot.Constants.HINGE_MOTOR_RIGHT_ID;
import static frc.robot.Constants.HINGE_STOW_POSITION;
import static frc.robot.Constants.INTAKE_EJECT_SPEED;
import static frc.robot.Constants.INTAKE_EJECT_TIME;
import static frc.robot.Constants.INTAKE_FEED_SPEED;
import static frc.robot.Constants.INTAKE_INTAKE_SPEED;
import static frc.robot.Constants.INTAKE_MOTOR_LEFT_ID;
import static frc.robot.Constants.INTAKE_MOTOR_RIGHT_ID;
import static frc.robot.Constants.INTAKE_SMART_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.PIDGains;

public class IntakeSubsystem extends SubsystemBase{
    
    private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("intake");

    private final DoublePublisher intakeSpeedPublisher = intakeTable.getDoubleTopic("intake-speed").publish();
    private final DoublePublisher intakeRpmPublisher = intakeTable.getDoubleTopic("intake-rpm").publish();
    private final DoublePublisher intakeDesiredSpeedPublisher = intakeTable.getDoubleTopic("intake-desired-speed").publish();

    private final StringPublisher intakeStatePublisher = intakeTable.getStringTopic("intake-state").publish();

    private final NetworkTable hingeTable = NetworkTableInstance.getDefault().getTable("hinge");

    private final DoublePublisher hingeSpeedPublisher = hingeTable.getDoubleTopic("hinge-speed").publish();
    private final DoublePublisher hingeRpmPublisher = hingeTable.getDoubleTopic("hinge-rpm").publish();
    private final DoublePublisher hingeDesiredPositionPublisher = hingeTable.getDoubleTopic("hinge-desired-position").publish();

    private final SparkMax intakeMotorRight;
    private final SparkMax intakeMotorLeft;
    private final SparkMaxConfig intakeMotorRightConfig;
    private final SparkMaxConfig intakeMotorLeftConfig;

    private final SparkClosedLoopController intakeMotorRightController;
    private final RelativeEncoder intakeMotorRightRelativeEncoder;
    private final PIDGains intakePidGains;

    private double desiredSpeed;

    private final SparkMax hingeMotorRight;
    private final SparkMax hingeMotorLeft;
    private final SparkMaxConfig hingeMotorRightConfig;
    private final SparkMaxConfig hingeMotorLeftConfig;

    private final SparkClosedLoopController hingeMotorRightController;
    private final RelativeEncoder hingeMotorRightRelativeEncoder;
    private final PIDGains hingePidGains;

    private double desiredPosition;

    private IntakeState intakeState;
    public static enum IntakeState {
        IDLE,
        INTAKE,
        FEED,
        EJECT,
        STOW
    };

    public IntakeSubsystem() {

        intakeMotorRight = new SparkMax(INTAKE_MOTOR_RIGHT_ID, BRUSHLESS);
        intakeMotorLeft = new SparkMax(INTAKE_MOTOR_LEFT_ID, BRUSHLESS);
        intakeMotorRightConfig = new SparkMaxConfig();
        intakeMotorLeftConfig = new SparkMaxConfig();

        intakeMotorRightController = intakeMotorRight.getClosedLoopController();
        intakeMotorRightRelativeEncoder = intakeMotorRight.getEncoder();
        intakePidGains = new PIDGains(1.0, 0.0, 0.0, 0.0);

        desiredSpeed = 0.0;

        hingeMotorRight = new SparkMax(HINGE_MOTOR_RIGHT_ID, BRUSHLESS);
        hingeMotorLeft = new SparkMax(HINGE_MOTOR_LEFT_ID, BRUSHLESS);
        hingeMotorRightConfig = new SparkMaxConfig();
        hingeMotorLeftConfig = new SparkMaxConfig();

        hingeMotorRightController = hingeMotorRight.getClosedLoopController();
        hingeMotorRightRelativeEncoder = hingeMotorRight.getEncoder();
        hingePidGains = new PIDGains(1.0, 0.0, 0.0, 0.0);

        desiredPosition = 0.0;

        intakeState = IntakeState.IDLE;

        configureMotors();
    }

    public void teleopInit() {}

    @Override
    public void periodic() {
        updateTelemetry();
    }

    @SuppressWarnings("removal")
    public void configureMotors() {
        intakeMotorRightConfig
            .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        intakeMotorRightConfig.closedLoop
            .p(intakePidGains.p)
            .i(intakePidGains.i)
            .d(intakePidGains.d)
            .velocityFF(intakePidGains.FF);
        
        intakeMotorRight.configure(
            intakeMotorRightConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
        
        intakeMotorLeftConfig
            .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
            .follow(INTAKE_MOTOR_RIGHT_ID)
            .idleMode(IdleMode.kCoast)
            .inverted(true);
        
        intakeMotorLeft.configure(
            intakeMotorLeftConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
        
        hingeMotorRightConfig
            .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        
        hingeMotorRightConfig.closedLoop
            .p(hingePidGains.p)
            .i(hingePidGains.i)
            .d(hingePidGains.d)
            .velocityFF(intakePidGains.FF);
        
        hingeMotorRight.configure(
            hingeMotorRightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
        hingeMotorLeftConfig
            .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
            .follow(HINGE_MOTOR_RIGHT_ID)
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        
        hingeMotorLeft.configure(
            hingeMotorLeftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {
        intakeSpeedPublisher.set(intakeMotorRight.getAppliedOutput());
        intakeRpmPublisher.set(intakeMotorRightRelativeEncoder.getVelocity());
        intakeDesiredSpeedPublisher.set(desiredSpeed);

        hingeSpeedPublisher.set(hingeMotorRight.getAppliedOutput());
        hingeRpmPublisher.set(hingeMotorRightRelativeEncoder.getVelocity());
        hingeDesiredPositionPublisher.set(desiredPosition);

        intakeStatePublisher.set(getState().toString());
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        intakeMotorRightController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> intakeMotorRightController.setSetpoint(0, ControlType.kDutyCycle));
    }

    public Command holdIntake() {
        return Commands.runOnce(() -> intakeMotorRightController.setSetpoint(intakeMotorRightRelativeEncoder.getPosition(), ControlType.kPosition));
    }

    public Command updateIntakeSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
        return Commands.runOnce(() -> setIntakeDutyCycle(this.desiredSpeed));
    }

    public void setHingePosition(double position) {
        hingeMotorRightController.setSetpoint(position, ControlType.kPosition);
    }

    public Command stopHinge() {
        return Commands.runOnce(() -> hingeMotorRightController.setSetpoint(0, ControlType.kDutyCycle));
    }

    public Command holdHinge() {
        return Commands.runOnce(() -> hingeMotorRightController.setSetpoint(hingeMotorRightRelativeEncoder.getPosition(), ControlType.kPosition));
    }

    public Command updateHingePosition(double desiredPosition) {
        this.desiredPosition = desiredPosition;
        return Commands.runOnce(() -> setHingePosition(this.desiredPosition));
    }

    public Command startIntakeCommand() {
        return new SequentialCommandGroup(
            setState(IntakeState.INTAKE),
            new ParallelCommandGroup(
                updateHingePosition(HINGE_INTAKE_POSITION),
                updateIntakeSpeed(INTAKE_INTAKE_SPEED)),
            stopHinge());
    }

    public Command feedIntakeCommand() {
        return new ParallelCommandGroup(
            setState(IntakeState.FEED),
            updateHingePosition(HINGE_FEED_POSITION),
            updateIntakeSpeed(INTAKE_FEED_SPEED));
    }

    public Command ejectIntakeCommand() {
        return new SequentialCommandGroup(
            setState(IntakeState.EJECT),
            holdIntake(),
            updateHingePosition(HINGE_EJECT_POSITION),
            updateIntakeSpeed(INTAKE_EJECT_SPEED),
            new WaitCommand(INTAKE_EJECT_TIME),
            stowIntakeCommand());
    }

    public Command stowIntakeCommand() {
        return new SequentialCommandGroup(
            setState(IntakeState.STOW),
            holdIntake(),
            updateHingePosition(HINGE_STOW_POSITION),
            stopIntake());
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> intakeState = state);
    }

    public IntakeState getState() {
        return intakeState;
    }
}