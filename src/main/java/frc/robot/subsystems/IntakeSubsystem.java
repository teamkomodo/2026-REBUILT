package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

public class IntakeSubsystem extends SubsystemBase {

    public static final boolean USE_ABSOLUTE_ENCODER = true; // Whether to use the absolute encoder for hinge position control (instead of relative encoder)

    /* NetworkTable */
    private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("intake");

    private final DoublePublisher intakeSpeedPublisher = intakeTable.getDoubleTopic("intake-speed").publish();
    private final DoublePublisher intakeRpmPublisher = intakeTable.getDoubleTopic("intake-rpm").publish();
    private final DoublePublisher intakeDesiredSpeedPublisher = intakeTable.getDoubleTopic("intake-desired-speed")
            .publish();

    private final StringPublisher intakeStatePublisher = intakeTable.getStringTopic("intake-state").publish();

    private final NetworkTable hingeTable = NetworkTableInstance.getDefault().getTable("hinge");

    private final DoublePublisher hingeSpeedPublisher = hingeTable.getDoubleTopic("hinge-speed").publish();
    private final DoublePublisher hingeRpmPublisher = hingeTable.getDoubleTopic("hinge-rpm").publish();
    private final DoublePublisher hingeDesiredPositionPublisher = hingeTable.getDoubleTopic("hinge-desired-position")
            .publish();
    // Publish absolute hinge encoder position
    private final DoublePublisher hingeAbsolutePositionPublisher = hingeTable.getDoubleTopic("hinge-absolute-position")
            .publish();

    /* ----- Intake ----- */
    // Intake motor and controller.
    private final SparkFlex intakeMotorLeft;
    private final SparkFlexConfig intakeMotorLeftConfig;

    private final SparkClosedLoopController intakeMotorLeftController;
    private final RelativeEncoder intakeMotorLeftRelativeEncoder;
    private final PIDGains intakePidGains;

    private double desiredSpeed;

    /* ----- Hinge ----- */
    // Hinge motor and controller (single motor)
    private final SparkMax hingeMotor;
    private final SparkMaxConfig hingeMotorConfig;

    private final SparkClosedLoopController hingeMotorController;
    private final RelativeEncoder hingeMotorRelativeEncoder;
    private final PIDGains hingePidGains;

    // Absolute encoder for hinge position
    private final SparkAbsoluteEncoder hingeAbsoluteEncoder;

    private double desiredPosition;

    private IntakeState intakeState;

    public static enum IntakeState {
        IDLE,
        INTAKE,
        FEED,
        EJECT,
        STOW
        // Add JAM_CLEAR?
    };

    public IntakeSubsystem() {
        // Intake motors and controllers
        intakeMotorLeft = new SparkFlex(INTAKE_MOTOR_LEFT_ID, BRUSHLESS);
        intakeMotorLeftConfig = new SparkFlexConfig();

        intakeMotorLeftController = intakeMotorLeft.getClosedLoopController();
        intakeMotorLeftRelativeEncoder = intakeMotorLeft.getEncoder();
        intakePidGains = new PIDGains(1.0, 0.0, 0.0, 0.0); // FIXME: Tune pid constants

        // Intake target variable
        desiredSpeed = 0.0;

        // Hinge motors and controllers and variables

        hingeMotor = new SparkMax(HINGE_MOTOR_ID, BRUSHLESS);
        hingeMotorConfig = new SparkMaxConfig();

        hingeMotorController = hingeMotor.getClosedLoopController();
        hingeMotorRelativeEncoder = hingeMotor.getEncoder();
        hingePidGains = new PIDGains(1.0, 0.0, 0.0, 0.0); // FIXME: Tune pid constants

        // Hinge target variable
        desiredPosition = 0.0;

        // Hinge absolute encoder
        if (USE_ABSOLUTE_ENCODER) {
            hingeAbsoluteEncoder = hingeMotor.getAbsoluteEncoder();
        } else {
            hingeAbsoluteEncoder = null;
        }

        // State variable
        intakeState = IntakeState.IDLE;

        // Configure motors
        configureMotors();
    }

    public void teleopInit() {
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void configureMotors() {
        intakeMotorLeftConfig
                .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast)
                .inverted(false);

        intakeMotorLeftConfig.closedLoop
                .p(intakePidGains.p)
                .i(intakePidGains.i)
                .d(intakePidGains.d);

        intakeMotorLeft.configure(
                intakeMotorLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        hingeMotorConfig
                .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake)
                .inverted(false);

        hingeMotorConfig.closedLoop
                .p(hingePidGains.p)
                .i(hingePidGains.i)
                .d(hingePidGains.d);

        hingeMotor.configure(
                hingeMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {
        intakeSpeedPublisher.set(intakeMotorLeft.getAppliedOutput());
        intakeRpmPublisher.set(intakeMotorLeftRelativeEncoder.getVelocity());
        intakeDesiredSpeedPublisher.set(desiredSpeed);

        hingeSpeedPublisher.set(hingeMotor.getAppliedOutput());
        hingeRpmPublisher.set(hingeMotorRelativeEncoder.getVelocity());
        hingeDesiredPositionPublisher.set(desiredPosition);
        if (USE_ABSOLUTE_ENCODER) {
            hingeAbsolutePositionPublisher.set(hingeAbsoluteEncoder.getPosition());
        }

        intakeStatePublisher.set(getState().toString());
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        intakeMotorLeftController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public Command stopIntake() {
        desiredSpeed = 0;
        return Commands.runOnce(() -> setIntakeDutyCycle(0.0));
    }

    public Command holdIntake() {
        desiredSpeed = 0;
        return Commands.runOnce(() -> intakeMotorLeftController
                .setSetpoint(intakeMotorLeftRelativeEncoder.getPosition(), ControlType.kPosition));
    }

    public Command startIntakeStowSpeed() {
        return updateIntakeSpeed(INTAKE_STOWING_SPEED);
    }

    public Command updateIntakeSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
        return Commands.runOnce(() -> setIntakeDutyCycle(this.desiredSpeed));
    }

    public void setHingePosition(double position) {
        hingeMotorController.setSetpoint(position, ControlType.kPosition);
    }

    public Command stopHinge() {
        return Commands.runOnce(() -> hingeMotorController.setSetpoint(0, ControlType.kDutyCycle));
    }

    public Command holdHinge() {
        // Prefer absolute encoder for a stable hinge hold position
        if (USE_ABSOLUTE_ENCODER) {
            desiredPosition = hingeAbsoluteEncoder.getPosition();
        } else {
            desiredPosition = hingeMotorRelativeEncoder.getPosition();
        }
        return Commands.runOnce(() -> hingeMotorController.setSetpoint(desiredPosition, ControlType.kPosition));
    }

    /** Return the hinge absolute encoder position. */
    public double getHingeAbsolutePosition() {
        return USE_ABSOLUTE_ENCODER? hingeAbsoluteEncoder.getPosition() : hingeMotorRelativeEncoder.getPosition();
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

    public Command stopIntakeCommand() {
        return stopIntake();
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> intakeState = state);
    }

    public IntakeState getState() {
        return intakeState;
    }
}