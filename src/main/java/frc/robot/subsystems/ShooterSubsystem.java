package frc.robot.subsystems;

import static frc.robot.Constants.BRUSHLESS;
import static frc.robot.Constants.SHOOTER_MOTOR_LEFT_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_RIGHT_ID;
import static frc.robot.Constants.SHOOTER_SMART_CURRENT_LIMIT;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;

/**
 * Small command helpers to integrate with SystemStateMachine expectations.
 */

public class ShooterSubsystem extends SubsystemBase{
    
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");

    private final DoublePublisher shooterSpeedPublisher = shooterTable.getDoubleTopic("shooter-speed").publish();
    private final DoublePublisher shooterRpmPublisher = shooterTable.getDoubleTopic("shooter-rpm").publish();
    private final DoublePublisher shooterDesiredSpeedPublisher = shooterTable.getDoubleTopic("shooter-desired-speed").publish();

    private final SparkMax shooterMotorRight;
    private final SparkMax shooterMotorLeft;
    private final SparkMaxConfig shooterMotorRightConfig;
    private final SparkMaxConfig shooterMotorLeftConfig;

    private final SparkClosedLoopController shooterMotorRightController;
    private final RelativeEncoder shooterMotorRightRelativeEncoder;
    private final PIDGains shooterPidGains;

    private double desiredSpeed;

    public ShooterSubsystem() {

        shooterMotorRight = new SparkMax(SHOOTER_MOTOR_RIGHT_ID, BRUSHLESS);
        shooterMotorLeft = new SparkMax(SHOOTER_MOTOR_LEFT_ID, BRUSHLESS);
        shooterMotorRightConfig = new SparkMaxConfig();
        shooterMotorLeftConfig = new SparkMaxConfig();

        shooterMotorRightController = shooterMotorRight.getClosedLoopController();
        shooterMotorRightRelativeEncoder = shooterMotorRight.getEncoder();
        shooterPidGains = new PIDGains(1.0, 0.0, 0.0, 0.0);

        desiredSpeed = 0.0;
        configureMotors();
    }

    public void teleopInit() {}

    @Override
    public void periodic() {
        updateTelemetry();
    }

    @SuppressWarnings("removal")
    public void configureMotors() {
        shooterMotorRightConfig
            .smartCurrentLimit(SHOOTER_SMART_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        shooterMotorRightConfig.closedLoop
            .p(shooterPidGains.p)
            .i(shooterPidGains.i)
            .d(shooterPidGains.d)
            .velocityFF(shooterPidGains.FF);
        
        shooterMotorRight.configure(
            shooterMotorRightConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
        
        shooterMotorLeftConfig
            .smartCurrentLimit(SHOOTER_SMART_CURRENT_LIMIT)
            .follow(SHOOTER_MOTOR_RIGHT_ID)
            .idleMode(IdleMode.kCoast)
            .inverted(true);
        
        shooterMotorLeft.configure(
            shooterMotorLeftConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {
        shooterSpeedPublisher.set(shooterMotorRight.getAppliedOutput());
        shooterRpmPublisher.set(shooterMotorRightRelativeEncoder.getVelocity());
        shooterDesiredSpeedPublisher.set(desiredSpeed);
    }

    public void setShooterDutyCycle(double dutyCycle) {
        shooterMotorRightController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void stopShooter() {
        shooterMotorRightController.setSetpoint(0, ControlType.kDutyCycle);
    }

    public Command stopShooterCommand() {
        return Commands.runOnce(this::stopShooter, this);
    }

    public void holdShooter() {
        shooterMotorRightController.setSetpoint(shooterMotorRightRelativeEncoder.getPosition(), ControlType.kPosition);
    }

    public Command updateShooterSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
        return Commands.runOnce(() -> setShooterDutyCycle(this.desiredSpeed));
    }

    public Command startShootingCommand() {
        // FIXME: need to call out to navx for this
        return updateShooterSpeed(0.1);
    }

    public Command stowShooterCommand() {
        // default stow behavior: stop shooter for now
        return stopShooterCommand();
    }
}
