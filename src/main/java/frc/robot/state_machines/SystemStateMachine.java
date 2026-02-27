package frc.robot.state_machines;

import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * SystemStateMachine: low-level system behaviors that coordinate subsystems.
 *
 * States:
 * TRAVEL, INTAKE, ALIGNING, SHOOT, INTAKE_AND_SHOOT, RESET, STOW, EMPTYING,
 * UNJAM
 *
 * Public API:
 * - RequestResult requestState(State target)
 * - Command requestStateCommand(State target)
 *
 * Keep onEntry/onExit fast. Schedule multi-step actions with Commands or the
 * subsystem's own async APIs.
 */
public class SystemStateMachine extends SubsystemBase {

    // NetworkTables telemetry
    private final NetworkTable systemTable = NetworkTableInstance.getDefault().getTable("system_state");
    private final StringPublisher systemStatePublisher = systemTable.getStringTopic("system-state").publish();
    private final StringPublisher systemLogPublisher = systemTable.getStringTopic("log").publish();

    public enum SystemState {
        OFF,
        TRAVEL,
        INTAKE,
        ALIGNING,
        SHOOT,
        SHOOT_ONCE,
        RESET,
        STOW,
        EMPTYING,
        MANUAL,
        UNJAM;

        /**
         * Defines the valid "next steps" from the current state.
         */
        public boolean canTransitionTo(SystemState from, SystemState to) {
            // Global escapes allowed from any state
            if (to == RESET || to == UNJAM || to == OFF) {
                return true;
            }

            return switch (from) {
                /* MANUAL mode is only allowed via automatic transition */
                case OFF -> Set.of(TRAVEL, RESET).contains(to);
                case TRAVEL -> Set.of(INTAKE, ALIGNING, STOW, EMPTYING, RESET, SHOOT, SHOOT_ONCE).contains(to);
                case INTAKE -> Set.of(TRAVEL, ALIGNING, STOW, EMPTYING, RESET, SHOOT, SHOOT_ONCE).contains(to);
                case ALIGNING -> Set.of(SHOOT, SHOOT_ONCE, TRAVEL, RESET).contains(to);
                case SHOOT -> Set.of(TRAVEL, EMPTYING, INTAKE, RESET).contains(to);
                case SHOOT_ONCE -> Set.of(TRAVEL, EMPTYING, INTAKE, RESET).contains(to);
                case RESET -> Set.of(TRAVEL, INTAKE, STOW).contains(to);
                case STOW -> Set.of(INTAKE, TRAVEL, RESET).contains(to);
                case EMPTYING -> Set.of(TRAVEL, INTAKE, RESET).contains(to);
                case MANUAL -> Set.of(TRAVEL, INTAKE, ALIGNING, SHOOT, RESET).contains(to);
                case UNJAM -> Set.of(TRAVEL, RESET).contains(to);
            };
        }
    }

    public enum RequestResult {
        ACCEPTED,
        ALREADY_IN_STATE,
        NO_SUCH_TRANSITION,
        REJECTED_GUARD
    }

    // Subsystem dependencies (replace/expand with your real subsystem APIs)
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final DrivetrainSubsystem drivetrain;

    // ManualActions class instance for this instance
    private final ManualActions manualActions;

    private volatile SystemState currentState = SystemState.TRAVEL;

    public SystemStateMachine(
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            DrivetrainSubsystem drivetrain) {

        this.intake = Objects.requireNonNull(intake);
        this.shooter = Objects.requireNonNull(shooter);
        this.indexer = Objects.requireNonNull(indexer);
        this.drivetrain = Objects.requireNonNull(drivetrain);
        this.manualActions = new ManualActions();
        configureTriggers();
    }

    private void configureTriggers() {
        // // If we are currently INTAKING and the indexer reports FULL, move to TRAVEL
        // // automatically
        // new Trigger(indexer::isIndexerFull)
        // .and(new Trigger(isInState(SystemState.INTAKE)))
        // .onTrue(requestState(SystemState.TRAVEL));

        // // If we are currently SHOOTING and the indexer reports EMPTY, move to TRAVEL
        // new Trigger(indexer::isEmpty)
        // .and(new Trigger(isInState(SystemState.SHOOT)))
        // .onTrue(requestState(SystemState.TRAVEL));

    }

    // Lightweight Context snapshot for guards
    public static class Context {
        public final SystemState currentState;

        public Context(SystemState currentState) {
            this.currentState = currentState;
        }
    }

    private Context snapshotContext() {
        return new Context(
                currentState);
    }

    public Command requestState(SystemState target) {
        return requestState(target, false);
    }

    public Command requestState(SystemState target, boolean force) {
        return Commands.defer(() -> {
            synchronized (this) {
                if (target == currentState)
                    return Commands.none();

                Context ctx = snapshotContext();

                if (!force && !currentState.canTransitionTo(ctx.currentState, target)) {
                    systemLogPublisher
                            .set(String.format("FINE: System transition %s -> %s rejected", currentState, target));
                    return Commands.none();
                }

                return performTransition(target, ctx);
            }
        }, Set.of(this));
    }

    public Command requestStateCommand(SystemState target) {
        // requestState already returns a Command that performs the transition when run
        return requestState(target);
    }

    public Command requestUnjam() {
        return requestState(SystemState.UNJAM);
    }

    private Command performTransition(SystemState target, Context ctx) {
        SystemState previous = currentState;

        return Commands.sequence(
                onExit(previous),
                cancelAll(),
                Commands.runOnce(() -> {
                    currentState = target;
                    systemLog(String.format("INFO: System: %s -> %s", previous, target));
                }),
                onEntry(target, snapshotContext())).withName("TransitionTo_" + target);
    }

    public Command cancelAll() {
        // We return a command that parallelizes all stop actions.
        return Commands.parallel(
                intake.stopIntake(),
                shooter.stopShooterCommand(),
                indexer.stopIndexerCommand()).withName("SystemCancelAll");
    }

    private Command onExit(SystemState previous) {
        return switch (previous) {
            case INTAKE, STOW, EMPTYING, SHOOT, ALIGNING, UNJAM, SHOOT_ONCE ->
                cancelAll();
            default -> Commands.none();
        };
    }

    private Command onEntry(SystemState target, Context ctx) {
        return (switch (target) {
            case INTAKE -> intake.startIntakeCommand(); // Deploys the intake and starts the rollers
            case STOW -> intake.stowIntakeCommand();
            case ALIGNING -> Commands.parallel(
                    // drivetrain.startVisionAlign(),
                    // FIXME: implement this method ^^^^ in DrivetrainSubsystem
                    shooter.startShootingCommand());
            // These commands can take a while to finish
            case SHOOT -> Commands.sequence(
                    // 1. Ensure shooter is commanded to spin
                    shooter.startShootingCommand(),
                    // 2. Wait until the sensors confirm we are at the target RPM
                    Commands.waitUntil(shooter::isAtTargetSpeed),
                    // Commands.waitUntil(drivetrain::isAlignedToTarget), // TODO: implement this method in DrivetrainSubsystem
                    // 3. Only then, run the indexer to fire the ball
                    shooter.startFeedingCommand()
                // Stop when the indexer is empty
                // Commands.waitUntil(() -> debouncer.calculate(indexer.isEmpty())),
                // shooter.stopFeedingCommand()
                );
            case SHOOT_ONCE -> Commands.sequence(
                    shooter.startShootingCommand(),
                    Commands.waitUntil(shooter::isAtTargetSpeed),
                    shooter.feedOnceCommand()
                );
            case EMPTYING -> Commands.parallel(intake.ejectIntakeCommand(), indexer.reverseCommand());
            case UNJAM -> Commands.parallel(indexer.reverseCommand(), intake.ejectIntakeCommand());
            case OFF -> Commands.parallel(intake.stopIntake(), shooter.stopShooterCommand(), indexer.stopIndexerCommand());
            case RESET -> Commands.parallel(intake.stopIntake(), intake.stowIntakeCommand(), shooter.stopShooterCommand(), indexer.stopIndexerCommand());
            default -> Commands.none();
        }).withName("SystemEntry_" + target);
    }

    // Public interface for manual controls

    public class ManualActions {

        // Intake Controls
        public Command intake() {
            return manualGate(intake.startIntakeCommand());
        }

        public Command intakeStop() {
            return manualGate(intake.stopIntakeCommand());
        }

        public Command intakeStow() {
            return manualGate(intake.stowIntakeCommand());
        }

        public Command intakeEject() {
            return manualGate(intake.ejectIntakeCommand());
        }

        // Shooter Controls
        public Command shootShort() {
            return manualGate(shooter.shortShotCommand());
        }

        public Command shootLong() {
            return manualGate(shooter.longShotCommand());
        }

        public Command shootPass() {
            return manualGate(shooter.passShotCommand());
        }

        public Command feedOnce() {
            return manualGate(shooter.feedOnceCommand());
        }

        public Command startFeeding() {
            return manualGate(shooter.startFeedingCommand());
        }

        public Command stopFeeding() {
            return manualGate(shooter.stopFeedingCommand());
        }

        // Indexer Controls
        public Command startIndexer() {
            return manualGate(indexer.startCommand());
        }

        private Command manualGate(Command action) {
            // Gate a command with a check for MANUAL state
            return action.onlyIf(() -> currentState == SystemState.MANUAL);
        }
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /** Publish lightweight telemetry about the system state. */
    public void updateTelemetry() {
        try {
            systemStatePublisher.set(currentState.toString());
        } catch (Exception e) {
            // ignore NetworkTables errors
        }
    }

    private void systemLog(String value) {
        systemLogPublisher.set(value);
        System.out.println("SystemStateMachine: " + value);
    }

    public SystemState getState() {
        return currentState;
    }

    public ManualActions getManualActions() {
        return manualActions;
    }

    public BooleanSupplier isInState(SystemState state) {
        return () -> currentState == state;
    }
}