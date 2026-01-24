package main.java.frc.robot.state_machines;

import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.state_machines.SystemStateMachine;
import frc.robot.state_machines.SystemStateMachine.SystemState;

/**
 * TeleopStateMachine: fine-grained teleop modes and transition policy.
 *
 * States:
 * - IDLE, STEAL, SCORE, MANUAL, RESET, CALIBRATE
 *
 * Public API:
 * - RequestResult requestState(TeleopState target)
 * - Command requestStateCommand(TeleopState target)
 * - void start(), void cancelAll(), void enterDisabled(), void enterTestMode()
 * - boolean isCalibrationComplete()
 *
 * Transition notes:
 * - RESET can be requested from any teleop state.
 * - CALIBRATE -> IDLE allowed only when calibrationComplete == true.
 * - CALIBRATE -> RESET allowed anytime.
 * - Most active TeleopState switches (IDLE/STEAL/SCORE/MANUAL) are permitted freely.
 * 
 * TODO: Wire automatic transitions.
 */
public class TeleopStateMachine {

    private static final Logger logger = Logger.getLogger(TeleopStateMachine.class.getName());

    public enum TeleopState {
        IDLE,
        STEAL,
        SCORE,
        MANUAL,
        RESET,
        CALIBRATE
    }

    public enum RequestResult {
        ACCEPTED,
        ALREADY_IN_STATE,
        NO_SUCH_TRANSITION,
        REJECTED_GUARD
    }

    // Dependencies (example): suppliers for operator input, and an optional
    // calibration completion supplier
    // If TeleopStateMachine itself manages calibration, it will update its internal
    // flag and expose isCalibrationComplete().
    private final SystemStateMachine systemSM;
    private final BooleanSupplier operatorOverrideSupplier; // used for any teleop-level gating if needed

    private volatile TeleopState currentState = TeleopState.IDLE;
    private volatile long stateEnteredTimestampMs = System.currentTimeMillis();

    public TeleopStateMachine(SystemStateMachine systemStateMachine, BooleanSupplier operatorOverrideSupplier) {
        this.systemSM = Objects.requireNonNull(systemStateMachine);
        this.operatorOverrideSupplier = Objects.requireNonNull(operatorOverrideSupplier);
    }

    // Snapshot used for guard evaluation
    public static class Context {
        public final TeleopState currentState;
        public final SystemState systemState;
        public final long stateEnteredTimestampMs;
        public final boolean calibrationComplete;
        public final boolean operatorOverride;

        public Context(TeleopState teleopState, SystemState systemState, long stateEnteredTimestampMs, boolean calibrationComplete,
                boolean operatorOverride) {
            this.currentState = teleopState;
            this.systemState = systemState;
            this.stateEnteredTimestampMs = stateEnteredTimestampMs;
            this.calibrationComplete = calibrationComplete;
            this.operatorOverride = operatorOverride;
        }
    }

    private Context snapshotContext() {
        return new Context(
            currentState, 
            systemSM.getState(),
            stateEnteredTimestampMs, 
            getCalibrationComplete(),
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
                    () -> String.format("Teleop transition %s -> %s rejected: %s (calComplete=%s, override=%s)",
                            ctx.currentState, target, allowed, ctx.calibrationComplete, ctx.operatorOverride));
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
                // allow CALIBRATE only if operatorOverride is true (optional) or you can remove
                // this guard.
                if (target == TeleopState.IDLE || target == TeleopState.STEAL || target == TeleopState.SCORE || target == TeleopState.MANUAL) {
                    return RequestResult.ACCEPTED;
                }
                if (target == TeleopState.CALIBRATE) {
                    // Require operator override to enter CALIBRATE during teleop
                    return ctx.operatorOverride ? RequestResult.ACCEPTED : RequestResult.REJECTED_GUARD;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case CALIBRATE:
                // From CALIBRATE: only allow RESET or IDLE (IDLE only when calibrationComplete)
                if (target == TeleopState.IDLE || target == TeleopState.RESET) {
                    return ctx.calibrationComplete || ctx.operatorOverride? RequestResult.ACCEPTED : RequestResult.REJECTED_GUARD;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case RESET:
                // After RESET completes, callers should move to IDLE or other active states;
                // allow immediate transitions to IDLE/STEAL/SCORE/MANUAL for simplicity.
                if (target == TeleopState.IDLE || target == TeleopState.STEAL || target == TeleopState.SCORE || target == TeleopState.MANUAL) {
                    return RequestResult.ACCEPTED;
                }
                if (target == TeleopState.CALIBRATE) {
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
            stateEnteredTimestampMs = System.currentTimeMillis();
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
            case CALIBRATE:
                // optionally do nothing (calibration results are kept in calibrationComplete
                // flag)
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
                // schedule steal routine or set drivetrain/intake targets
                break;

            case SCORE:
                // schedule scoring behavior
                break;

            case MANUAL:
                // give direct operator control
                break;

            case CALIBRATE:
                // start calibration procedure
                startCalibrationProcedure();
                break;

            case RESET:
                // start reset procedure and when done, callers will transition away
                startResetProcedure();
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

    // Example async starts (implementations are placeholders)
    private void startCalibrationProcedure() {
        // Kick off asynchronous calibration routine.
        // When finished, call markCalibrationComplete().
        // Do not block here.
    }

    private void startResetProcedure() {
        // Kick off reset routine; when complete, teleop controller or higher-level code
        // may requestState(IDLE)
        // Do not block here.
    }

    private boolean getCalibrationComplete() {
        return systemSM.calibrationCompleteSupplier().getAsBoolean();
    }

    public BooleanSupplier calibrationCompleteSupplier() {
        return systemSM.calibrationCompleteSupplier();
    }

    // Diagnostics
    public TeleopState getCurrentState() {
        return currentState;
    }

    public long getStateEnteredTimestampMs() {
        return stateEnteredTimestampMs;
    }
}