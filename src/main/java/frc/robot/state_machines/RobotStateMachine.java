package frc.robot.state_machines;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.logging.Level;
import java.util.logging.Logger;
import frc.robot.state_machines.TeleopStateMachine;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;
import frc.robot.state_machines.SystemStateMachine;
import frc.robot.state_machines.SystemStateMachine.SystemState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * RobotStateMachine for the 2026 FRC season.
*/

/**
 * RobotStateMachine: coarse robot modes and transition policy.
 *
 * Public API:
 * - RequestResult requestState(RobotState target)
 * - Command requestStateCommand(RobotState target) // factory that returns a
 * schedulable command
 *
 * Transition rules (implemented in canTransitionTo):
 * - DISABLED -> only TELEOP or AUTO
 * - From AUTO -> TELEOP allowed 
 * - DISABLED is allowed from any state
 * - Other transitions are explicit in the switch below
 *
 * Transition semantics:
 * - requestState is non-blocking, idempotent (early-return if already in
 * target).
 * - Accepted transitions immediately interrupt current activity (onExit) and
 * run onEntry for the new state.
 * - onEntry should schedule long-running work (Commands) rather than performing
 * it inline.
 * 
 * TODO: Wire automatic transitions.
 */
public class RobotStateMachine {

    private static final Logger logger = Logger.getLogger(RobotStateMachine.class.getName());

    public enum RobotState {
        DISABLED,
        AUTO,
        TELEOP
        // FULL_RESET,
        // TEST
    }

    public enum RequestResult {
        ACCEPTED,
        ALREADY_IN_STATE,
        NO_SUCH_TRANSITION,
        REJECTED_GUARD
    }

    // Dependencies: teleop and system state machines, intake subsystem, operator
    // override supplier, etc.
    private final TeleopStateMachine teleopSM;
    private final SystemStateMachine systemSM;
    private final BooleanSupplier operatorOverrideSupplier;

    private volatile RobotState currentState = RobotState.DISABLED;

    public RobotStateMachine(
            TeleopStateMachine teleopSM,
            SystemStateMachine systemSM,
            BooleanSupplier operatorOverrideSupplier) {
        this.teleopSM = Objects.requireNonNull(teleopSM);
        this.systemSM = Objects.requireNonNull(systemSM);
        this.operatorOverrideSupplier = Objects.requireNonNull(operatorOverrideSupplier);
    }

    /**
     * Snapshot of runtime values used for guard evaluation. Keep this lightweight.
     */
    public static class Context {
        public final RobotState robotState;
        public final TeleopState teleopState;
        public final SystemState systemState;
        public final boolean operatorOverride;

        public Context(RobotState robotState, TeleopState teleopState, SystemState systemState, boolean operatorOverride) {
            this.robotState = robotState;
            this.teleopState = teleopState;
            this.systemState = systemState;
            this.operatorOverride = operatorOverride;
        }
    }

    private Context snapshotContext() {
        return new Context(
                currentState,
                teleopSM.getState(),
                systemSM.getState(),
                operatorOverrideSupplier.getAsBoolean()
        );
    }

    /**
     * Request a state transition. Idempotent and returns a RequestResult with a
     * simple reason.
     */
    public synchronized RequestResult requestState(RobotState target) {
        if (target == currentState) {
            return RequestResult.ALREADY_IN_STATE;
        }

        Context ctx = snapshotContext();

        RequestResult can = canTransitionTo(ctx, target);
        if (can != RequestResult.ACCEPTED) {
            logReject(ctx, target, can);
            return can;
        }

        // perform transition: interrupt current activity, set new state, run entry
        // actions
        performTransition(target, ctx);
        return RequestResult.ACCEPTED;
    }

    /**
     * Factory that returns a Command wrapper for requestState.
     */
    public Command requestStateCommand(RobotState target) {
        return new InstantCommand(() -> requestState(target));
    }

    private void logReject(Context ctx, RobotState target, RequestResult reason) {
        logger.log(Level.FINE, () -> String.format("Transition %s -> %s rejected: %s (override=%s)",
                ctx.robotState, target, reason, ctx.operatorOverride));
    }

    /**
     * Canonical small switch-based transition guard. Add special-case guards here.
     */
    private RequestResult canTransitionTo(Context ctx, RobotState target) {
        // DISABLED is always allowed
        if (target == RobotState.DISABLED) {
            return RequestResult.ACCEPTED;
        }

        switch (ctx.robotState) {
            case DISABLED:
                if (target == RobotState.TELEOP || target == RobotState.AUTO) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case AUTO:
                // From AUTO: TELEOP allowed; DISABLED allowed
                if (target == RobotState.TELEOP || target == RobotState.DISABLED) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case TELEOP:
                // TELEOP may go to DISABLED
                if (target == RobotState.DISABLED) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;
            default:
                return RequestResult.NO_SUCH_TRANSITION;
        }
    }

    /*
     * performTransition: interrupt current activity, run exit, set state, run
     * entry.
     * Keep onExit/onEntry fast; schedule long-running work from onEntry using
     * Commands.
     */
    private void performTransition(RobotState target, Context ctx) {
        RobotState previous = currentState;
        try {
            // 1) synchronous cleanup / interrupt of previous state
            try {
                onExit(previous);
            } catch (Exception e) {
                logger.log(Level.WARNING, "Exception during onExit of " + previous, e);
            }

            // Cancel or preempt any long-running Commands owned by the previous mode.
            cancelModeActivities(previous);

            // 2) set new state
            currentState = target;
            logger.log(Level.INFO, () -> String.format("Robot transitioned from %s to %s", previous, target));

            // 3) run entry actions (schedule commands, enable subsystems, etc.)
            try {
                onEntry(target, snapshotContext());
            } catch (Exception e) {
                logger.log(Level.WARNING, "Exception during onEntry of " + target, e);
            }

        } finally {
            // any metrics or publishable state
        }
    }

    // Lightweight cancel hook: ensure current mode's long-running activities are
    // stopped immediately.
    private void cancelModeActivities(RobotState previous) {
        switch (previous) {
            case TELEOP:
            case AUTO:
            case DISABLED:
                cancelAll();
                break;
            default:
                // nothing to cancel
                break;
        }
    }

    /**
     * onExit hook: stop motors, clear intents relevant to the previous top-level
     * state.
     * Keep fast and deterministic; prefer scheduling cleanup tasks as Commands
     * rather than long blocking work here.
     */
    private void onExit(RobotState previous) {
        switch (previous) {
            case TELEOP:
            case AUTO:
            case DISABLED:
            default:
                // nothing to do
                break;
        }
    }

    /**
     * onEntry hook: enable subsystems, schedule Commands for the new top-level
     * state behavior.
     * Do not block.
     */
    private void onEntry(RobotState target, Context ctx) {
        switch (target) {
            case DISABLED:
                systemSM.requestState(SystemState.OFF);
                teleopSM.requestState(TeleopState.IDLE);
                systemSM.cancelAll();
                teleopSM.cancelAll();
                break;

            case AUTO:
                // systemSM.prepareForAuto(); // TODO: fix this line; no such method
                teleopSM.requestState(TeleopState.IDLE);
                // Optionally schedule autonomous Command here or externally
                break;

            case TELEOP:
                // Activate teleop state machine; teleopSM will control subsystems via its own
                // API.
                teleopSM.requestState(TeleopState.SCORE); // TODO: choose appropriate initial teleop state
                // systemSM.armForTeleop(); // TODO: fix this line; no such method
                break;

            default:
                break;
        }
    }

    public void cancelAll() {
        // Cancel all activities in both state machines
        systemSM.cancelAll();
        teleopSM.cancelAll();
    }

    public RobotState getCurrentState() {
        return currentState;
    }
}