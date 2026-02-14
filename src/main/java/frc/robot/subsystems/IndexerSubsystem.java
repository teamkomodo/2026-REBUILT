package frc.robot.subsystems;

import static frc.robot.Constants.BRUSHLESS;
import static frc.robot.Constants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.INDEXER_SMART_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.PIDGains;

public class IndexerSubsystem extends SubsystemBase{
    
    private final NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("indexer");
    
    private final DoublePublisher indexerSpeedPublisher = indexerTable.getDoubleTopic("indexer-speed").publish();
    private final BooleanPublisher indexerFullSensorPublisher = indexerTable.getBooleanTopic("indexer-full-sensor").publish();
    private final BooleanPublisher indexerEmptySensorPublisher = indexerTable.getBooleanTopic("indexer-ready-sensor").publish();

    private final StringPublisher indexerStatePublisher = indexerTable.getStringTopic("indexer-state").publish();

    private final SparkMax indexerMotor;
    private final SparkMaxConfig indexerMotorConfig;

    private final SparkClosedLoopController indexerMotorController;
    private final RelativeEncoder indexerMotorRelativeEncoder;
    private final PIDGains indexerPidGains;
    
    // Beambreaks for indexerFull and indexerEmpty
    private final DigitalInput beamBreakIsFull;
    private final DigitalInput beamBreakIsEmpty;


    private double desiredSpeed;

    private IndexerState indexerState;
    public static enum IndexerState {
        IDLE,
        PIECE_READY,
        AGITATING,
        INDEXER_FULL
    };

    public IndexerSubsystem() {

        indexerMotor = new SparkMax(INDEXER_MOTOR_ID, BRUSHLESS);
        indexerMotorConfig = new SparkMaxConfig();

        indexerMotorController = indexerMotor.getClosedLoopController();
        indexerMotorRelativeEncoder = indexerMotor.getEncoder();
        indexerPidGains = new PIDGains(1.0, 0.0, 0.0, 0.0); //FIXME
        
        desiredSpeed = 0.0;

        // Initialize beam-break sensors (DIO channels in Constants)
        beamBreakIsFull = new DigitalInput(Constants.INDEXER_BEAM_BREAK_FULL_CHANNEL);
        beamBreakIsEmpty = new DigitalInput(Constants.INDEXER_BEAM_BREAK_EMPTY_CHANNEL);

        indexerState = IndexerState.IDLE;
    }

    // Basic control helpers used by the SystemStateMachine. These are minimal
    // and can be expanded to use closed-loop controllers.
    public IndexerState getState() {
        return indexerState;
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> desiredSpeed = 0);
    }

    public Command reverseCommand() {
        return Commands.runOnce(() -> desiredSpeed = -Math.abs(desiredSpeed == 0 ? Constants.INDEXER_SPEED : desiredSpeed));
    }

    public void teleopInit() {}

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void configureMotors() {
        indexerMotorConfig
            .smartCurrentLimit(INDEXER_SMART_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        
        indexerMotorConfig.closedLoop
            .p(indexerPidGains.p)
            .i(indexerPidGains.i)
            .d(indexerPidGains.d)
            .feedForward.sv(0.0, indexerPidGains.FF);
        
        indexerMotor.configure(
            indexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {
        indexerSpeedPublisher.set(indexerMotor.get());
        indexerStatePublisher.set(indexerState.toString());
        // Publish beam-break sensor states (true means circuit closed / sensor triggered)
        try {
            indexerFullSensorPublisher.set(beamBreakIsFull.get());
        } catch (Exception e) {
            indexerFullSensorPublisher.set(false);
        }
        try {
            indexerEmptySensorPublisher.set(beamBreakIsEmpty.get());
        } catch (Exception e) {
            indexerEmptySensorPublisher.set(false);
        }
    }

    /** Returns true if the indexer full beam-break is triggered. */
    public boolean isIndexerFull() {
        return beamBreakIsFull.get();
    }

    /** Returns true if the indexer ready (piece present) beam-break is triggered. */
    public boolean isPieceReady() {
        return beamBreakIsEmpty.get();
    }
}
