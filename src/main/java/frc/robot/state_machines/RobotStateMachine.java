package frc.robot.state_machines;

import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;
import frc.robot.state_machines.SystemStateMachine.SystemState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
 */
public class RobotStateMachine extends SubsystemBase {

    // NetworkTables telemetry
    private final NetworkTable robotTable = NetworkTableInstance.getDefault().getTable("robot_state");
    private final StringPublisher robotStatePublisher = robotTable.getStringTopic("robot-state").publish();
    private final StringPublisher robotLogPublisher = robotTable.getStringTopic("log").publish();

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

        public Context(RobotState robotState, TeleopState teleopState, SystemState systemState,
                boolean operatorOverride) {
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
                operatorOverrideSupplier.getAsBoolean());
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void updateTelemetry() {
        try {
            robotStatePublisher.set(currentState.toString());
        } catch (Exception e) {
            // ignore NetworkTables errors
        }
    }

    private void robotLog(String value) {
        robotLogPublisher.set(value);
        System.out.println("RobotStateMachine: " + value);
    }

    /**
     * Request a state transition. Returns a Command.
     */
    public Command requestState(RobotState target) {
        // We use defer so that 'snapshotContext' and 'canTransitionTo'
        // are evaluated when the command starts, not at startup.
        return Commands.defer(() -> {
            if (target == currentState) {
                return Commands.none();
            }

            Context ctx = snapshotContext();
            RequestResult result = canTransitionTo(ctx, target);

            if (result != RequestResult.ACCEPTED) {
                return Commands.runOnce(() -> logReject(ctx, target, result));
            }

            // Return the transition command we built earlier
            return performTransition(target, ctx);
        }, Set.of()); // No subsystem requirements needed for the logic itself
    }

    /**
     * Factory that returns a Command wrapper for requestState.
     */
    public Command requestStateCommand(RobotState target) {
        return requestState(target);
    }

    private void logReject(Context ctx, RobotState target, RequestResult reason) {
        robotLog(String.format("FINE: Transition %s -> %s rejected: %s (override=%s)",
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

    /**
     * Returns a Command that handles the state transition sequence.
     * This ensures the robot successfully exits the old state before
     * updating internal variables and starting the new state behavior.
     */
    private Command performTransition(RobotState target, Context ctx) {
        RobotState previous = currentState;

        return Commands.sequence(
                // 1) Run the exit command for the previous state
                onExit(previous)
                        .handleInterrupt(() -> robotLog("WARNING: onExit interrupted for " + previous)),

                // 2) Cleanup existing long-running activities
                new InstantCommand(() -> cancelModeActivities(previous)),

                // 3) Update the state variable & log (The "Hook")
                new InstantCommand(() -> {
                    currentState = target;
                    robotLog(String.format("INFO: Robot transitioned from %s to %s", previous, target));
                }),

                // 4) Run the entry command for the new state
                onEntry(target, snapshotContext())
                        .handleInterrupt(() -> robotLog("WARNING: onEntry interrupted for " + target)))
                .withName("Transition_" + previous + "_to_" + target);
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
    private Command onExit(RobotState previous) {
        return switch (previous) {
            case TELEOP -> Commands.none();
            case AUTO -> Commands.none();
            case DISABLED -> Commands.none();
            default -> Commands.none();
        };
    }

    /**
     * onEntry hook: enable subsystems, schedule Commands for the new top-level
     * state behavior.
     * Do not block.
     */

    private Command onEntry(RobotState target, Context ctx) {
        return switch (target) {
            case DISABLED -> Commands.parallel(
                    systemSM.requestState(SystemState.OFF),
                    teleopSM.requestState(TeleopState.IDLE),
                    Commands.runOnce(() -> {
                        systemSM.cancelAll();
                    })).withName("Entry_Disabled");

            case AUTO -> Commands.parallel(
                    teleopSM.requestState(TeleopState.IDLE)
                // Add autonomous routine command here if needed
                ).withName("Entry_Auto");

            case TELEOP -> Commands.parallel(
                    teleopSM.requestState(TeleopState.SCORE),
                    systemSM.requestState(SystemState.TRAVEL)).withName("Entry_Teleop");

            default -> Commands.none();
        };
    }

    public void cancelAll() {
        // Cancel all activities in both state machines
        systemSM.cancelAll();
    }

    public RobotState getCurrentState() {
        return currentState;
    }
}