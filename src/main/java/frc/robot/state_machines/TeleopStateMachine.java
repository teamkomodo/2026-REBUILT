package frc.robot.state_machines;

import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
 * - Most active TeleopState switches (IDLE/STEAL/SCORE/MANUAL) are permitted
 * freely.
 * 
 */
public class TeleopStateMachine extends SubsystemBase {

    // NetworkTables telemetry
    private final NetworkTable teleopTable = NetworkTableInstance.getDefault().getTable("teleop_state");
    private final StringPublisher teleopStatePublisher = teleopTable.getStringTopic("teleop-state").publish();
    private final StringPublisher teleopLogPublisher = teleopTable.getStringTopic("log").publish();
    private final DoublePublisher teleopMatchStagePublisher = teleopTable.getDoubleTopic("match-stage").publish();

    // For testing, this being true will always permit those transitions that are
    // normally restricted to automatic triggers.
    public static final boolean DONT_RESTRICT_AUTO_TRANSITIONS = true;
    public final boolean TEST_AUTO_SWITCHING = false;
    public final boolean TEST_OUR_HUB_STARTS_ACTIVE = true;

    public enum TeleopState {
        IDLE,
        STEAL,
        SCORE,
        MANUAL,
        RESET;

        /**
         * Defines the valid transitions for the Teleop state machine.
         */
        @SuppressWarnings("unused")
        public boolean canTransitionTo(TeleopState target, Context ctx) {
            // Global escape: You can always RESET
            if (target == RESET) {
                return true;
            }

            // Handle transition checks
            boolean canTransition = false;
            if (ctx.operatorOverride) {
                // Handle operator override cases (very permissive)
                canTransition = canTransition || switch (this) {
                    case IDLE -> Set.of(STEAL, SCORE, MANUAL).contains(target);
                    case STEAL -> Set.of(IDLE, SCORE, MANUAL).contains(target);
                    case SCORE -> Set.of(IDLE, STEAL, MANUAL).contains(target);
                    case MANUAL -> Set.of(IDLE, STEAL, SCORE).contains(target);
                    case RESET -> target == IDLE;

                    default -> false;
                };
            } else if (ctx.isAutomaticTransition || DONT_RESTRICT_AUTO_TRANSITIONS) {
                // Handle automatically triggered transitions (more permissive)
                canTransition = canTransition || switch (this) {
                    case IDLE -> Set.of(STEAL, SCORE).contains(target);
                    case STEAL -> Set.of(IDLE, SCORE).contains(target);
                    case SCORE -> Set.of(IDLE, STEAL).contains(target);
                    case MANUAL -> Set.of(IDLE).contains(target);
                    case RESET -> target == IDLE;

                    default -> false;
                };
            } else {
                // Normal cases (somewhat restrictive)
                canTransition = canTransition || switch (this) {
                    case IDLE -> Set.of(STEAL, SCORE).contains(target);
                    case STEAL -> Set.of(IDLE).contains(target);
                    case SCORE -> Set.of(IDLE).contains(target);
                    case MANUAL -> Set.of(IDLE).contains(target);
                    case RESET -> target == IDLE;

                    default -> false;
                };
            }

            return canTransition;
        }
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
    private String receivedGameData;
    public volatile int matchStage = 0; // 0 = early match, 1 - 4 = alliance shifts, 5 = endgame

    private volatile TeleopState currentState = TeleopState.IDLE;

    public TeleopStateMachine(SystemStateMachine systemStateMachine, BooleanSupplier operatorOverrideSupplier) {
        this.systemSM = Objects.requireNonNull(systemStateMachine);
        this.operatorOverrideSupplier = Objects.requireNonNull(operatorOverrideSupplier);
    }

    public TeleopStateMachine(SystemStateMachine systemSM, BooleanSupplier operatorOverrideSupplier,
            boolean START_IN_MANUAL) {
        this(systemSM, operatorOverrideSupplier);
        if (START_IN_MANUAL) {
            this.currentState = TeleopState.MANUAL;
        }
    }

    // Snapshot used for guard evaluation
    public static class Context {
        public final TeleopState currentState;
        public final SystemState systemState;
        public final boolean operatorOverride;
        public boolean isAutomaticTransition = false; // Is overridden in requestState

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
                operatorOverrideSupplier.getAsBoolean());
    }

    // Public API used by RobotStateMachine and other code
    public Command requestState(TeleopState target) {
        return requestState(target, false);
    }

    public Command requestState(TeleopState target, boolean isAutomaticTransition) {
        return Commands.defer(() -> {
            synchronized (TeleopStateMachine.this) {
                if (target == currentState && currentState != TeleopState.RESET) {
                    return Commands.none();
                }

                Context ctx = snapshotContext();
                ctx.isAutomaticTransition = isAutomaticTransition;

                // The enum logic handles the "allowed" check
                if (!currentState.canTransitionTo(target, ctx)) {
                    teleopLog(String.format("FINE: Teleop transition %s -> %s rejected", currentState, target));
                    return Commands.none();
                }

                return performTransition(target, ctx);
            }
        }, Set.of());
    }

    public Command requestStateCommand(TeleopState target) {
        return requestState(target);
    }

    private Command performTransition(TeleopState target, Context ctx) {
        TeleopState previous = currentState;

        return Commands.sequence(
                // 1) Cleanup previous state (returns a Command)
                onExit(previous),
                // 2) Stop active teleop routines (returns a Command)
                systemSM.cancelAll(),
                // 3) Update state variable and log
                Commands.runOnce(() -> {
                    currentState = target;
                    teleopLog(String.format("INFO: Teleop transitioned %s -> %s", previous, target));
                }),
                // 4) Execute new state entry (returns a Command)
                onEntry(target, snapshotContext())).withName("TeleopTransition_" + target);
    }

    private Command onExit(TeleopState previous) {
        return switch (previous) {
            case IDLE -> Commands.none();
            default -> systemSM.cancelAll();
        };
    }

    private Command onEntry(TeleopState target, Context ctx) {
        return switch (target) {
            case IDLE -> Commands.none();
            case STEAL -> systemSM.requestState(SystemState.INTAKE); // TODO: Consider doing INTAKE_AND_SHOOT
            case SCORE -> Commands.none(); // TODO: schedule scoring behavior
            case MANUAL -> systemSM.requestState(SystemState.MANUAL); // give direct operator control
            case RESET -> systemSM.requestState(SystemState.RESET);
        };
    }

    // Timed state switch command
    public Command teleopMasterCommand() {
        return Commands.sequence(
                // First 10 seconds: SCORE
                setMatchStage(0),
                requestState(TeleopState.SCORE, true),

                // Determine active hub while waiting (10s)
                new ParallelDeadlineGroup(
                        Commands.waitSeconds(10.0),
                        Commands.waitUntil(canDetermineActiveHubSupplier)),

                // Run the timer and switch states accordingly
                // Require the TeleopStateMachine for the lifetime of this deferred command
                // so other commands that would require this subsystem cannot cancel it.
                Commands.defer(() -> {
                    // boolean ourHubInactiveFirst = isOurHubInactiveFirstSupplier.getAsBoolean();
                    boolean ourHubInactiveFirst = isOurHubInactiveFirst().orElse(false);
                    // If the code failed to determine the active hub, ourHubInactiveFirst defaults
                    // to false, starting the robot in SCORE state
                    // TODO: Review the above behavior and make sure it is acceptable

                    // Simple testing support
                    if (TEST_AUTO_SWITCHING) {
                        ourHubInactiveFirst = !TEST_OUR_HUB_STARTS_ACTIVE;
                    }

                    // Set the states
                    TeleopState initial = ourHubInactiveFirst ? TeleopState.STEAL : TeleopState.SCORE;
                    TeleopState opposite = ourHubInactiveFirst ? TeleopState.SCORE : TeleopState.STEAL;

                    // Run the timer
                    return Commands.sequence(
                            // Shift 1 (25s)
                            setMatchStage(1),
                            requestState(initial, true), Commands.waitSeconds(25.0),
                            // Shift 2 (25s)
                            setMatchStage(2),
                            requestState(opposite, true), Commands.waitSeconds(25.0),
                            // Shift 3 (25s)
                            setMatchStage(3),
                            requestState(initial, true), Commands.waitSeconds(25.0),
                            // Shift 4 (25s)
                            setMatchStage(4),
                            requestState(opposite, true), Commands.waitSeconds(25.0),
                            // Final: SCORE until match ends
                            setMatchStage(5),
                            requestState(TeleopState.SCORE, true));
                }, Set.of(this))).withName("TeleopMasterTimeline")
                .ignoringDisable(false);
        // FIXME: !!!! Make sure other commands will not cancel this !!!!
    }

    public BooleanSupplier canDetermineActiveHubSupplier = () -> {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            char activeAlliance = gameData.charAt(0);
            receivedGameData = gameData;
            return activeAlliance == 'R' || activeAlliance == 'B';
        } else {
            return false;
        }
    };

    public Optional<Boolean> isOurHubInactiveFirst() {
        String gameData;
        // Ensure gameData (mostly)
        if (receivedGameData == null || receivedGameData.isEmpty()) {
            gameData = DriverStation.getGameSpecificMessage();
        } else {
            gameData = receivedGameData;
        }
        // Return true if the inactive hub is ours
        Optional<Alliance> isOnRedAlliance = DriverStation.getAlliance();
        if (gameData.length() > 0 && isOnRedAlliance.isPresent()) {
            char inactiveAlliance = gameData.charAt(0);
            boolean onRedAlliance = isOnRedAlliance.get() == Alliance.Red;
            return Optional.of(
                    onRedAlliance ? inactiveAlliance == 'R' : inactiveAlliance == 'B');
        } else {
            return Optional.empty();
        }
    };

    private Command setMatchStage(int stage) {
        return Commands.runOnce(() -> {
            matchStage = stage;
        });
    }

    public void enterDisabled() {
        // Put teleop state machine into a safe disabled state.
        systemSM.cancelAll();
        currentState = TeleopState.IDLE;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /** Publish lightweight teleop telemetry. */
    public void updateTelemetry() {
        try {
            teleopStatePublisher.set(currentState.toString());
            teleopMatchStagePublisher.set(matchStage);
        } catch (Exception e) {
            // ignore NetworkTables errors
        }
    }

    private void teleopLog(String value) {
        teleopLogPublisher.set(value);
        System.out.println("TeleopStateMachine: " + value);
    }

    public TeleopState getState() {
        return currentState;
    }
}