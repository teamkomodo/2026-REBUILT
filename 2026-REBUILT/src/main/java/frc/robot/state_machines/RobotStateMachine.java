// package main.java.frc.robot.state_machines; // VSCode unhappy if uncommented for Elijah

/**
    Planned state machine levels: Robot > Strategy > System  > subsystem
 	    RobotStateMachine has these states:
    DISABLED, CALIBRATE, AUTO, TELEOP (uses StrategySM), (FULL_RESET), (TEST) 
        TeleopStateMachine has these states:
    IDLE, STEAL, SCORE, MANUAL, and RESET.
        SystemStateMachine has these states:
    TRAVEL, INTAKE, ALIGNING, SHOOT, and INTAKE_AND_SHOOT, UNJAM (intake and indexer).
        IntakeSubsystem:
    STOW, DEPLOY, INTAKE, LOCK, EJECT, JAM_CLEAR
    API: deploy(), stow(), start(), stop(), lock(), eject(), isStowed(), getState()
        ShooterSubsystem:
    STOW, IDLE, AIM, SPINUP, READY, FIRE, FIRING, COOLDOWN (optional)
    API: aim(double angle), aim(), start(), stop(), isReady(), shootOnce(), startShooting(), stopShooting(), isStowed(), getState()
        IndexerSubsystem:
    IDLE, AGITATE, REVERSE, JAM_CLEAR
    API: start(), stop(), reverse(), isEmpty(), isFull(), getState()
        DrivetrainSubsystem:
    DRIVE, various ALIGN_* states, PATH_FOLLOW (optional)
    APIs: not sure, but at least getState()
        Class count: seven (7)
    Not going to start with a TEST state or FULL_RESET (may these add later).
    Definitely log all state machine states (including subsystems) to shuffleboard. Yes, we should add a function to stop all subsystems, and call on entry into DISABLED (have had problems in the past with immediate robot motion on re-enable)
    Will add transition gates in code Error handling is not the point of state machine. The point is to coordinate the subsystems to prevent collisions. Operator override button to bypass transition checks (only in non-dangerous scenario).
    */

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

/**
 * Top-level robot state machine.
 * States: DISABLED, CALIBRATE, AUTO, TELEOP
 * DISABLED always forces stopAllSubsystems().
 */
public class RobotStateMachine {
    public enum RobotState { DISABLED, CALIBRATE, AUTO, TELEOP }

    private RobotState current = RobotState.DISABLED;

    private final TeleopStateMachine teleop; // strategy machine used in TELEOP
    private final SystemStateMachine system;
    private final Runnable stopAllSubsystems;
    private final BooleanSupplier allowAutoSupplier; // optional gating for AUTO

    public RobotStateMachine(
            TeleopStateMachine teleop,
            SystemStateMachine system,
            Runnable stopAllSubsystems,
            BooleanSupplier allowAutoSupplier) {
        this.teleop = teleop;
        this.system = system;
        this.stopAllSubsystems = stopAllSubsystems;
        this.allowAutoSupplier = allowAutoSupplier;
        publish();
    }

    public RobotState getState() {
        return current;
    }

    /**
     * Call in Robot.periodic()
     */
    public void periodic() {
        // Enforce DISABLED behavior
        if (DriverStation.isDisabled() && current != RobotState.DISABLED) {
            enterDisabled();
            return;
        }

        // Minimal publish
        publish();
        // Optionally dispatch periodic to children depending on mode
        if (current == RobotState.TELEOP) {
            teleop.periodic();
            system.periodic();
        } else if (current == RobotState.AUTO) {
            system.periodic();
            // auton-specific logic could run here or in system
        }
    }

    private void publish() {
        SmartDashboard.putString("RobotState", current.name());
        SmartDashboard.putString("TeleopState", teleop.getState().name());
        SmartDashboard.putString("SystemState", system.getState().name());
    }

    private void enterDisabled() {
        current = RobotState.DISABLED;
        stopAllSubsystems.run(); // immediate neutralization
        // Keep teleop/system state as-is or reset as desired
        publish();
    }

    // External requests (operator/RobotContainer can call)
    public void requestEnterCalibrate() {
        if (current == RobotState.DISABLED) {
            current = RobotState.CALIBRATE;
            publish();
        }
    }

    public void requestEnterAuto() {
        if (current == RobotState.CALIBRATE || current == RobotState.DISABLED) {
            if (allowAutoSupplier == null || allowAutoSupplier.getAsBoolean()) {
                current = RobotState.AUTO;
                publish();
            }
        }
    }

    public void requestEnterTeleop() {
        if (current == RobotState.AUTO || current == RobotState.CALIBRATE || current == RobotState.DISABLED) {
            current = RobotState.TELEOP;
            publish();
        }
    }
}
