package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.PIDGains;

public class IntakeSubsystem extends SubsystemBase {

    public static final boolean USE_ABSOLUTE_ENCODER = false; // Whether to use the absolute encoder for hinge position
                                                              // control (instead of relative encoder)

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
    private final DoublePublisher hingeRelativePositionPublisher = hingeTable.getDoubleTopic("hinge-relative-position")
            .publish();

    /* ----- Intake ----- */
    // Intake motor and controller.
    private final SparkFlex intakeMotorLeft;
    private final SparkFlexConfig intakeMotorLeftConfig;
    private final SparkFlex intakeMotorRight;
    private final SparkFlexConfig intakeMotorRightConfig;

    private final SparkClosedLoopController intakeMotorLeftController;
    private final RelativeEncoder intakeMotorLeftRelativeEncoder;
    private final PIDGains intakePidGains;

    private double desiredSpeed;
    private boolean returnToIntaking = false;

    /* ----- Hinge ----- */
    // Hinge motor and controller (single motor)
    private final SparkFlex hingeMotor;
    private final SparkFlexConfig hingeMotorConfig;

    private final SparkClosedLoopController hingeMotorController;
    private final RelativeEncoder hingeMotorRelativeEncoder;
    private final PIDGains hingePidGains;

    // Absolute encoder for hinge position
    private final SparkAbsoluteEncoder hingeAbsoluteEncoder;

    private double desiredPositionRotations;
    private final static double MAX_ALLOWED_ERROR_ROTATIONS = 0.1; // Rotations?

    private IntakeState intakeState;

    public static enum IntakeState {
        IDLE,
        INTAKE,
        FEED,
        EJECT,
        STOW,
        JIGGLE
        // Add JAM_CLEAR?
    };

    public IntakeSubsystem() {
        // Intake motors and controllers
        intakeMotorLeft = new SparkFlex(INTAKE_MOTOR_LEFT_ID, BRUSHLESS);
        intakeMotorLeftConfig = new SparkFlexConfig();
        intakeMotorRight = new SparkFlex(INTAKE_MOTOR_RIGHT_ID, BRUSHLESS);
        intakeMotorRightConfig = new SparkFlexConfig();

        intakeMotorLeftController = intakeMotorLeft.getClosedLoopController();
        intakeMotorLeftRelativeEncoder = intakeMotorLeft.getEncoder();
        intakePidGains = new PIDGains(1.0, 0.000, 0.0, 0.00); // FIXME: Tune pid constants

        // Intake target variable
        desiredSpeed = 0.0;

        // Hinge motors and controllers and variables

        hingeMotor = new SparkFlex(HINGE_MOTOR_ID, BRUSHLESS);
        hingeMotorConfig = new SparkFlexConfig();

        hingeMotorController = hingeMotor.getClosedLoopController();
        hingeMotorRelativeEncoder = hingeMotor.getEncoder();
        hingePidGains = new PIDGains(1.0, 0.0002, 0.01, 0.001); // FIXME: Tune pid constants

        // Hinge target variable
        desiredPositionRotations = 0.0;

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
                //.smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast)
                .inverted(true);

        intakeMotorLeftConfig.closedLoop
                .p(intakePidGains.p)
                .i(intakePidGains.i)
                .d(intakePidGains.d)
                .velocityFF(intakePidGains.FF);

        intakeMotorLeft.configure(
                intakeMotorLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        intakeMotorRightConfig
                .smartCurrentLimit(INTAKE_SMART_CURRENT_LIMIT)
                .follow(INTAKE_MOTOR_LEFT_ID, true)
                .idleMode(IdleMode.kCoast);

        intakeMotorRight.configure(
                intakeMotorRightConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        hingeMotorConfig
                .smartCurrentLimit(HINGE_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast)
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
        hingeDesiredPositionPublisher.set(desiredPositionRotations);
        if (USE_ABSOLUTE_ENCODER) {
            hingeAbsolutePositionPublisher.set(hingeAbsoluteEncoder.getPosition());
        }
        hingeRelativePositionPublisher.set(hingeMotorRelativeEncoder.getPosition());
        intakeStatePublisher.set(getState().toString());
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        intakeMotorLeftController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void setIntakeVelocity(double velocity) {
        intakeMotorLeftController.setSetpoint(velocity, ControlType.kVelocity);
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
        return updateIntakeDutyCycle(INTAKE_STOWING_SPEED);
    }

    public Command updateIntakeDutyCycle(double desiredSpeed) {
        return Commands.runOnce(() -> {
            System.out.println("======RUNNING INTAKE");
            this.desiredSpeed = desiredSpeed;
            setIntakeDutyCycle(desiredSpeed);
        });
    }

    public Command updateIntakeSpeed(double desiredSpeed) {
        return Commands.runOnce(() -> {
            System.out.println("======RUNNING INTAKE");
            this.desiredSpeed = desiredSpeed;
            setIntakeDutyCycle(desiredSpeed);
        });
    }

    public Command updateIntakeRollerRPM(double desiredRPM) {
        return Commands.runOnce(() -> {
            System.out.println("======RUNNING INTAKE RPM");
            setIntakeVelocity(desiredRPM * INTAKE_GEAR_RATIO);
        });
    }

    public void setIntakeRelativePosition(double relativePosition) {
        System.out.println("========== SETTING HINGE TO RELATIVE POSITION: " + relativePosition);
        System.out.println("========== HINGE POS: " + intakeMotorLeftRelativeEncoder.getPosition());
        hingeMotorController.setSetpoint(
                intakeMotorLeftRelativeEncoder.getPosition() + relativePosition, ControlType.kPosition);
    }

    public void setHingePosition(double position) {
        System.out.println("========== SETTING HINGE TO POSITION: " + position);
        System.out.println("========== HINGE POS: " + getHingeEncoderAbsolutePositionRotations());
        hingeMotorController.setSetpoint(position, ControlType.kPosition);
    }

    public void setHingeRelativePosition(double relativePosition) {
        System.out.println("========== SETTING HINGE TO RELATIVE POSITION: " + relativePosition);
        System.out.println("========== HINGE POS: " + getHingeEncoderAbsolutePositionRotations());
        hingeMotorController.setSetpoint(
                getHingeEncoderAbsolutePositionRotations() + relativePosition, ControlType.kPosition);
    }

    public Command moveHingeToPositionAndWait(double position) {
        return Commands.sequence(
                Commands.runOnce(() -> setHingePosition(position)),
                Commands.race(
                        Commands.waitSeconds(3),
                        Commands.waitUntil(this::isAtDesiredPosition)));
    }

    public Command runHingeAtDutyCycleForSeconds(double dutyCycle, double seconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    updateHingeDutyCycle(dutyCycle);
                }),
                Commands.waitSeconds(seconds),
                Commands.runOnce(() -> updateHingeDutyCycle(0)));
    }

    public void updateHingeDutyCycle(double dutycycle) {
        hingeMotorController.setSetpoint(dutycycle, ControlType.kDutyCycle);
    }

    public Command stopHinge() {
        return Commands.runOnce(() -> updateHingeDutyCycle(0));
    }

    public Command holdHinge() {
        // Prefer absolute encoder for a stable hinge hold position
        desiredPositionRotations = getHingeEncoderAbsolutePositionRotations();
        return Commands
                .runOnce(() -> hingeMotorController.setSetpoint(desiredPositionRotations, ControlType.kPosition));
    }

    /** Return the hinge absolute encoder position. */
    public double getHingeEncoderAbsolutePositionRotations() {
        return USE_ABSOLUTE_ENCODER ? hingeAbsoluteEncoder.getPosition() : hingeMotorRelativeEncoder.getPosition();
    }

    public Command deployIntake() {
        return runHingeAtDutyCycleForSeconds(HINGE_DEPLOY_DUTY_CYCLE, 2);
    }

    public Command deployIntakeAuto() {
        return runHingeAtDutyCycleForSeconds(EVIL_HINGE_DUTY_CYCLE, 4);
    }

    public Command stowIntake() {
        return runHingeAtDutyCycleForSeconds(HINGE_STOW_DUTY_CYCLE, 1);
    }

    public Command jiggleWithReverse() {
        return new SequentialCommandGroup(
            setState(IntakeState.JIGGLE),
            setReturnToIntaking(),
            new RepeatCommand(
                new SequentialCommandGroup(
                    Commands.waitSeconds(0.1),
                    Commands.runOnce(() -> setIntakeDutyCycle(INTAKE_JIGGLE_FORWARD_DUTYCYCLE)),
                    Commands.waitSeconds(0.3),
                    Commands.runOnce(() -> setIntakeDutyCycle(INTAKE_JIGGLE_REVERSE_DUTYCYCLE))
                )
            )
        );
    }

    public Command setReturnToIntaking() {
        return Commands.runOnce(() -> {
            if (getIntakeDesiredSpeed() == 0) {
                returnToIntaking = false;
            } else {
                returnToIntaking = true;
            }
        });
    }

    public Command stopJiggle() {
        if (returnToIntaking) {
            return updateIntakeDutyCycle(INTAKE_INTAKE_SPEED);
        } else {
            return stopIntake();
        }
    }

    public Command stopJiggleFixed() {
        return Commands.either(
            updateIntakeDutyCycle(INTAKE_INTAKE_SPEED),
            stopIntake(),
            () -> returnToIntaking
        );
    }

    public Command updateHingePosition(double desiredPosition) {
        this.desiredPositionRotations = desiredPosition;
        return Commands.runOnce(() -> setHingePosition(this.desiredPositionRotations));
    }

    public Command startIntakeCommand() {
        return new SequentialCommandGroup(
                setState(IntakeState.INTAKE),
                updateIntakeDutyCycle(INTAKE_INTAKE_SPEED),
                deployIntake());
    }

    public Command startIntakeAutoCommand() {
        return new SequentialCommandGroup(
                setState(IntakeState.INTAKE),
                updateIntakeSpeed(INTAKE_INTAKE_SPEED),
                Commands.runOnce(() -> holdHinge()));
                // deployIntakeAuto());
    }

    public Command feedIntakeCommand() {
        return new ParallelCommandGroup(
                setState(IntakeState.FEED),
                updateIntakeDutyCycle(INTAKE_INTAKE_SPEED));
    }

    public Command ejectIntakeCommand() {
        return new SequentialCommandGroup(
                setState(IntakeState.EJECT),
                updateIntakeDutyCycle(INTAKE_EJECT_SPEED),
                new WaitCommand(INTAKE_EJECT_TIME),
                stopIntake());
    }

    public Command stowIntakeCommand() {
        return new SequentialCommandGroup(
                setState(IntakeState.STOW),
                stowIntake());
    }

    public Command stopIntakeCommand() {
        return stopIntake();
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> intakeState = state);
    }

    public boolean isAtDesiredPosition() {
        return Math.abs(
                getHingeEncoderAbsolutePositionRotations() - desiredPositionRotations) < MAX_ALLOWED_ERROR_ROTATIONS;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public double getIntakeDesiredSpeed() {
        return desiredSpeed;
    }
}