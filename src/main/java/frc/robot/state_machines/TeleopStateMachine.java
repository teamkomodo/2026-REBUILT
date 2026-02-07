package frc.robot.state_machines;

import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.state_machines.SystemStateMachine;
import frc.robot.state_machines.SystemStateMachine.SystemState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TeleopStateMachine: fine-grained teleop modes and transition policy.
 *
 * States:
 * - IDLE, STEAL, SCORE, MANUAL, RESET
 *
 * Public API:
 * - RequestResult requestState(TeleopState target)
 * - Command requestStateCommand(TeleopState target)
 * - void start(), void cancelAll(), void enterDisabled(), void enterTestMode()
 *
 * Transition notes:
 * - RESET can be requested from any teleop state.
 * - Most active TeleopState switches (IDLE/STEAL/SCORE/MANUAL) are permitted freely.
 * 
 * TODO: Wire automatic transitions.
 */
public class TeleopStateMachine extends SubsystemBase {

    private static final Logger logger = Logger.getLogger(TeleopStateMachine.class.getName());

    public enum TeleopState {
        IDLE,
        STEAL,
        SCORE,
        MANUAL,
        RESET
    }

    public enum RequestResult {
        ACCEPTED,
        ALREADY_IN_STATE,
        NO_SUCH_TRANSITION,
        REJECTED_GUARD
    }

    // Dependencies (example): suppliers for operator input, and an optional
    private final SystemStateMachine systemSM;
    private final BooleanSupplier operatorOverrideSupplier; // used for any teleop-level gating if needed

    private volatile TeleopState currentState = TeleopState.IDLE;

    public TeleopStateMachine(SystemStateMachine systemStateMachine, BooleanSupplier operatorOverrideSupplier) {
        this.systemSM = Objects.requireNonNull(systemStateMachine);
        this.operatorOverrideSupplier = Objects.requireNonNull(operatorOverrideSupplier);
    }

    // Snapshot used for guard evaluation
    public static class Context {
        public final TeleopState currentState;
        public final SystemState systemState;
        public final boolean operatorOverride;

        public Context(TeleopState teleopState, SystemState systemState, boolean operatorOverride) {
            this.currentState = teleopState;
            this.systemState = systemState;
            this.operatorOverride = operatorOverride;
        }
    }

    private Context snapshotContext() {
        return new Context(
            currentState, 
            systemSM.getState(),
            operatorOverrideSupplier.getAsBoolean()
        );
    }

    // Public API used by RobotStateMachine and other code
    public synchronized RequestResult requestState(TeleopState target) {
        if (target == currentState) {
            return RequestResult.ALREADY_IN_STATE;
        }

        Context ctx = snapshotContext();
        RequestResult allowed = canTransitionTo(ctx, target);
        if (allowed != RequestResult.ACCEPTED) {
            logger.log(Level.FINE,
                    () -> String.format("Teleop transition %s -> %s rejected: %s (override=%s)",
                            ctx.currentState, target, allowed, ctx.operatorOverride));
            return allowed;
        }

        performTransition(target, ctx);
        return RequestResult.ACCEPTED;
    }

    public Command requestStateCommand(TeleopState target) {
        return new InstantCommand(() -> requestState(target));
    }

    // Simple switch-based transition guard
    private RequestResult canTransitionTo(Context ctx, TeleopState target) {
        // RESET is allowed from anywhere
        if (target == TeleopState.RESET) {
            return RequestResult.ACCEPTED;
        }

        switch (ctx.currentState) {
            case IDLE:
            case STEAL:
            case SCORE:
            case MANUAL:
                // From active teleop states, allow switching among IDLE/STEAL/SCORE/MANUAL
                // freely,
                // this guard.
                if (target == TeleopState.IDLE || target == TeleopState.STEAL || target == TeleopState.SCORE || target == TeleopState.MANUAL) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case RESET:
                // After RESET completes, callers should move to IDLE or other active states;
                // allow immediate transitions to IDLE/STEAL/SCORE/MANUAL for simplicity.
                if (target == TeleopState.IDLE || target == TeleopState.STEAL || target == TeleopState.SCORE || target == TeleopState.MANUAL) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            default:
                return RequestResult.NO_SUCH_TRANSITION;
        }
    }

    private void performTransition(TeleopState target, Context ctx) {
        TeleopState previous = currentState;
        try {
            // orderly exit from previous state
            try {
                onExit(previous);
            } catch (Exception e) {
                logger.log(Level.WARNING, "Exception during teleop onExit for " + previous, e);
            }

            // cancel any long-running teleop activities
            cancelAll();

            // set new state
            currentState = target;
            logger.log(Level.INFO, () -> String.format("Teleop transitioned %s -> %s", previous, target));

            // run entry actions (non-blocking)
            try {
                onEntry(target, snapshotContext());
            } catch (Exception e) {
                logger.log(Level.WARNING, "Exception during teleop onEntry for " + target, e);
            }
        } finally {
            // publish telemetry if desired
        }
    }

    // Keep these fast; schedule Commands for complex behaviors
    private void onExit(TeleopState previous) {
        switch (previous) {
            case IDLE:
            case STEAL:
            case SCORE:
            case MANUAL:
                // stop any intents, release actuators if needed
                // e.g., shooter.stopSpin(), intake.stop(), etc. (not implemented here)
                break;
            case RESET:
                // ensure reset routines are stopped
                break;
            default:
                break;
        }
    }

    private void onEntry(TeleopState target, Context ctx) {
        switch (target) {
            case IDLE:
                // clear operator targets; set safe profiles
                break;

            case STEAL:
                systemSM.requestState(SystemState.INTAKE_AND_SHOOT);
                break;

            case SCORE:
                // schedule scoring behavior
                break;

            case MANUAL:
                // give direct operator control
                break;

            case RESET:
                systemSM.requestState(SystemState.RESET);
                break;
        }
    }

    // Public control methods used by RobotStateMachine and other code
    public void start() {
        // Start teleop internals (register listeners, enable periodic updates, etc.)
        // This should be non-blocking.
    }

    public void cancelAll() {
        // Cancel or stop any running teleop commands/routines (non-blocking).
        // If you use a CommandScheduler, cancel teleop-owned commands here.
    }

    public void enterDisabled() {
        // Put teleop state machine into a safe disabled state.
        cancelAll();
        currentState = TeleopState.IDLE;
    }

    public TeleopState getState() {
        return currentState;
    }
}