package frc.robot.subsystems;

import static frc.robot.Constants.BRUSHLESS;
import static frc.robot.Constants.INTAKE_MOTOR_LEFT_ID;
import static frc.robot.Constants.INTAKE_MOTOR_RIGHT_ID;
import static frc.robot.Constants.INTAKE_SMART_CURRENT_LIMIT;
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

public class IntakeSubsystem extends SubsystemBase{
    
    private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("intake");

    private final DoublePublisher intakeSpeedPublisher = intakeTable.getDoubleTopic("intake-speed").publish();
    private final DoublePublisher intakeRpmPublisher = intakeTable.getDoubleTopic("intake-rpm").publish();
    private final DoublePublisher intakeDesiredSpeedPublisher = intakeTable.getDoubleTopic("intake-desired-speed").publish();

    private final SparkMax intakeMotorRight;
    private final SparkMax intakeMotorLeft;
    private final SparkMaxConfig intakeMotorRightConfig;
    private final SparkMaxConfig intakeMotorLeftConfig;

    private final SparkClosedLoopController intakeMotorRightController;
    private final RelativeEncoder intakeMotorRightRelativeEncoder;
    private final PIDGains intakePidGains;

    private double desiredSpeed;

    // private final SparkMax hingeMotorRight;
    // private final SparkMax hingeMotorLeft;
    // private final SparkMaxConfig hingeMotorRightConfig;
    // private final SparkMaxConfig hingeMotorLeftConfig;

    // private final SparkClosedLoopController hingeMotorRightController;
    // private final RelativeEncoder hingeMotorRightRelativeEncoder;
    // private final PIDGains hingePidGains;

    public IntakeSubsystem() {

        intakeMotorRight = new SparkMax(INTAKE_MOTOR_RIGHT_ID, BRUSHLESS);
        intakeMotorLeft = new SparkMax(INTAKE_MOTOR_LEFT_ID, BRUSHLESS);
        intakeMotorRightConfig = new SparkMaxConfig();
        intakeMotorLeftConfig = new SparkMaxConfig();

        intakeMotorRightController = intakeMotorRight.getClosedLoopController();
        intakeMotorRightRelativeEncoder = intakeMotorRight.getEncoder();
        intakePidGains = new PIDGains(1.0, 0.0, 0.0, 0.0);

        desiredSpeed = 0.0;

        //hingeMotorRight = new SparkMax(IN);
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
    }

    public void updateTelemetry() {
        intakeSpeedPublisher.set(intakeMotorRight.getAppliedOutput());
        intakeRpmPublisher.set(intakeMotorRightRelativeEncoder.getVelocity());
        intakeDesiredSpeedPublisher.set(desiredSpeed);
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        intakeMotorRightController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void stopIntake() {
        intakeMotorRightController.setSetpoint(0, ControlType.kDutyCycle);
    }

    public void holdIntake() {
        intakeMotorRightController.setSetpoint(intakeMotorRightRelativeEncoder.getPosition(), ControlType.kPosition);
    }

    public Command updateIntakeSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
        return Commands.runOnce(() -> setIntakeDutyCycle(this.desiredSpeed));
    }
}
