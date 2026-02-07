package frc.robot.subsystems;

import static frc.robot.Constants.BRUSHLESS;
import static frc.robot.Constants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.INDEXER_SMART_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;

public class IndexerSubsystem extends SubsystemBase{
    
    private final NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("indexer");
    
    private final DoublePublisher indexerSpeedPublisher = indexerTable.getDoubleTopic("indexer-speed").publish();
    private final BooleanPublisher indexerFullSensorPublisher = indexerTable.getBooleanTopic("indexer-full-sensor").publish();
    private final BooleanPublisher indexerReadySensorPublisher = indexerTable.getBooleanTopic("indexer-ready-sensor").publish();

    private final StringPublisher indexerStatePublisher = indexerTable.getStringTopic("indexer-state").publish();

    private final SparkMax indexerMotor;
    private final SparkMaxConfig indexerMotorConfig;

    private final SparkClosedLoopController indexerMotorController;
    private final RelativeEncoder indexerMotorRelativeEncoder;
    private final PIDGains indexerPidGains;

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

        indexerState = IndexerState.IDLE;
    }

    public void teleopInit() {}

    @Override
    public void periodic() {
        updateTelemetry();
    }

    @SuppressWarnings("removal")
    public void configureMotors() {
        indexerMotorConfig
            .smartCurrentLimit(INDEXER_SMART_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        
        indexerMotorConfig.closedLoop
            .p(indexerPidGains.p)
            .i(indexerPidGains.i)
            .d(indexerPidGains.d)
            .velocityFF(indexerPidGains.FF);
        
        indexerMotor.configure(
            indexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {

    }
}
