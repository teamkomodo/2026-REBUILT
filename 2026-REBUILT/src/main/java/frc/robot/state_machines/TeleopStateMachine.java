package main.java.frc.robot.state_machines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

/**
 * Teleop/Strategy level state machine.
 * States: IDLE, STEAL, SCORE, MANUAL, RESET
 */
public class TeleopStateMachine {
    public enum TeleopState { IDLE, STEAL, SCORE, MANUAL, RESET }

    private TeleopState current = TeleopState.IDLE;
    private final SystemStateMachine system;
    private final BooleanSupplier operatorOverride; // allow bypass of guards

    public TeleopStateMachine(SystemStateMachine system, BooleanSupplier operatorOverride) {
        this.system = system;
        this.operatorOverride = operatorOverride;
        publish();
    }

    public TeleopState getState() {
        return current;
    }

    public void periodic() {
        publish();
        // Optionally map teleop state -> requested system behaviors
        // Example: if STEAL then request INTAKE on system machine
        switch (current) {
            case STEAL:
                system.requestState(SystemStateMachine.SystemState.INTAKE);
                break;
            case SCORE:
                system.requestState(SystemStateMachine.SystemState.ALIGNING);
                break;
            case MANUAL:
                // MANUAL may leave direct commands to operator; system can be IDLE
                break;
            case RESET:
                system.requestState(SystemStateMachine.SystemState.TRAVEL);
                break;
            case IDLE:
            default:
                system.requestState(SystemStateMachine.SystemState.TRAVEL);
                break;
        }
    }

    private void publish() {
        SmartDashboard.putString("TeleopState", current.name());
        SmartDashboard.putBoolean("TeleopOverride", operatorOverride != null && operatorOverride.getAsBoolean());
    }

    // External requests (from operator bindings)
    public void requestState(TeleopState requested) {
        // Optionally gate transitions here (use operatorOverride to bypass)
        // Simple acceptance for now:
        this.current = requested;
        publish();
    }

    public void resetToIdle() {
        this.current = TeleopState.IDLE;
        publish();
    }
}