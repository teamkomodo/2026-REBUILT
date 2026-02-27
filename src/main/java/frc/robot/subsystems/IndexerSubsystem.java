package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.PIDGains;

public class IndexerSubsystem extends SubsystemBase {

    private final NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("indexer");

    private final DoublePublisher indexerSpeedPublisher = indexerTable.getDoubleTopic("indexer-speed").publish();
    // private final BooleanPublisher indexerFullSensorPublisher =
    // indexerTable.getBooleanTopic("indexer-full-sensor")
    // .publish();
    // private final BooleanPublisher indexerReadySensorPublisher =
    // indexerTable.getBooleanTopic("indexer-ready-sensor")
    // .publish();

    private final StringPublisher indexerStatePublisher = indexerTable.getStringTopic("indexer-state").publish();

    private final SparkFlex indexerMotor;
    private final SparkFlexConfig indexerMotorConfig;

    private final SparkClosedLoopController indexerMotorController;
    private final RelativeEncoder indexerMotorRelativeEncoder;
    private final PIDGains indexerPidGains;

    // Beambreaks for indexerFull and indexerEmpty
    // private final DigitalInput beamBreakIsFull;
    // private final DigitalInput beamBreakIsReady;

    private double desiredSpeed;

    private IndexerState indexerState;

    public static enum IndexerState {
        IDLE,
        PIECE_READY,
        AGITATING,
        INDEXER_FULL,
        REVERSE
    };

    public IndexerSubsystem() {

        indexerMotor = new SparkFlex(INDEXER_MOTOR_ID, BRUSHLESS);
        indexerMotorConfig = new SparkFlexConfig();

        indexerMotorController = indexerMotor.getClosedLoopController();
        indexerMotorRelativeEncoder = indexerMotor.getEncoder();
        indexerPidGains = new PIDGains(1.0, 0.0, 0.0, 0.0); // FIXME

        desiredSpeed = 0.0;

        // Initialize beam-break sensors (DIO channels in Constants)
        // beamBreakIsFull = new DigitalInput(INDEXER_BEAM_BREAK_FULL_CHANNEL);
        // beamBreakIsReady = new DigitalInput(INDEXER_BEAM_BREAK_READY_CHANNEL);

        indexerState = IndexerState.IDLE;

        configureMotors();
    }

    // Basic control helpers used by the SystemStateMachine. These are minimal
    // and can be expanded to use closed-loop controllers.
    public IndexerState getState() {
        return indexerState;
    }

    public Command stopIndexerCommand() {
        return Commands.runOnce(() -> {
            indexerState = IndexerState.IDLE;
            updateSpeed(0);
        }, this);
    }

    public Command reverseCommand() {
        return Commands.runOnce(() -> {
            indexerState = IndexerState.REVERSE;
            updateSpeed(-Math.abs(desiredSpeed == 0 ? INDEXER_DUTYCYCLE : desiredSpeed));
        }, this);
    }

    public Command startCommand() {
        return Commands.runOnce(() -> {
            indexerState = IndexerState.AGITATING;
            System.out.println("==== Starting indexer");
            updateSpeed(INDEXER_DUTYCYCLE);
        }, this);
    }

    public void updateSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
        setDutyCycle(desiredSpeed);
    }

    public void setSpeed(double speed) {
        indexerMotorController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void setDutyCycle(double dutyCycle) {
        indexerMotorController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void teleopInit() {
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void configureMotors() {
        indexerMotorConfig
                .smartCurrentLimit(INDEXER_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast)
                .inverted(true);

        indexerMotorConfig.closedLoop
                .p(indexerPidGains.p)
                .i(indexerPidGains.i)
                .d(indexerPidGains.d);

        indexerMotor.configure(
                indexerMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {
        indexerSpeedPublisher.set(indexerMotor.get());
        indexerStatePublisher.set(indexerState.toString());
        // Publish beam-break sensor states (true means circuit closed / sensor
        // triggered)
        // try {
        // indexerFullSensorPublisher.set(beamBreakIsFull.get());
        // } catch (Exception e) {
        // indexerFullSensorPublisher.set(false);
        // }
        // try {
        // indexerReadySensorPublisher.set(beamBreakIsReady.get());
        // } catch (Exception e) {
        // indexerReadySensorPublisher.set(false);
        // }
    }

    // /** Returns true if the indexer full beam-break is triggered. */
    // public boolean isIndexerFull() {
    // return beamBreakIsFull.get();
    // }

    // /**
    // * Returns true if the indexer ready (piece present) beam-break is triggered.
    // */
    // public boolean isPieceReady() {
    // return beamBreakIsReady.get();
    // }

    // public boolean isEmpty() {
    // return !isPieceReady() && !isIndexerFull();
    // }
}
