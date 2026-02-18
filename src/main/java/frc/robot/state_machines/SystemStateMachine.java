package frc.robot.state_machines;

import static frc.robot.Constants.*;

import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * SystemStateMachine: low-level system behaviors that coordinate subsystems.
 *
 * States:
 * TRAVEL, INTAKE, ALIGNING, SHOOT, INTAKE_AND_SHOOT, RESET, DEPLOY, EMPTYING,
 * UNJAM
 *
 * Public API:
 * - RequestResult requestState(State target)
 * - Command requestStateCommand(State target)
 *
 * Keep onEntry/onExit fast. Schedule multi-step actions with Commands or the
 * subsystem's own async APIs.
 */
public class SystemStateMachine {

    private static final Logger logger = Logger.getLogger(SystemStateMachine.class.getName());

    public enum SystemState {
        OFF,
        TRAVEL,
        INTAKE,
        ALIGNING,
        SHOOT,
        RESET,
        DEPLOY,
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
                case OFF -> to == TRAVEL;
                case TRAVEL -> Set.of(INTAKE, ALIGNING, DEPLOY, EMPTYING).contains(to);
                case INTAKE -> Set.of(TRAVEL, ALIGNING, DEPLOY, EMPTYING).contains(to);
                case ALIGNING -> Set.of(SHOOT, TRAVEL).contains(to);
                case SHOOT -> Set.of(TRAVEL, EMPTYING, INTAKE).contains(to);
                case RESET -> Set.of(TRAVEL, INTAKE, DEPLOY).contains(to);
                case DEPLOY -> Set.of(INTAKE, TRAVEL).contains(to);
                case EMPTYING -> Set.of(TRAVEL, INTAKE).contains(to);
                case MANUAL -> Set.of(TRAVEL, INTAKE, ALIGNING, SHOOT).contains(to);
                case UNJAM -> to == TRAVEL;
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
    private final Debouncer debouncer;

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
        this.debouncer = new Debouncer(BEAMBREAK_DEBOUNCE_DURATION, Debouncer.DebounceType.kRising);
        configureTriggers();
    }

    private void configureTriggers() {
        // If we are currently INTAKING and the indexer reports FULL, move to TRAVEL
        // automatically
        new Trigger(indexer::isIndexerFull)
                .and(new Trigger(isInState(SystemState.INTAKE)))
                .onTrue(requestState(SystemState.TRAVEL));

        // If we are currently SHOOTING and the indexer reports EMPTY, move to TRAVEL
        new Trigger(indexer::isEmpty)
                .and(new Trigger(isInState(SystemState.SHOOT)))
                .onTrue(requestState(SystemState.TRAVEL));

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
        return Commands.defer(() -> {
            synchronized (this) {
                if (target == currentState)
                    return Commands.none();

                Context ctx = snapshotContext();

                if (!currentState.canTransitionTo(ctx.currentState, target)) {
                    logger.log(Level.FINE, () -> String.format(
                            "System transition %s -> %s rejected", currentState, target));
                    return Commands.none();
                }

                return performTransition(target, ctx);
            }
        }, Set.of());
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
                    logger.log(Level.INFO, String.format("System: %s -> %s", previous, target));
                }),
                onEntry(target, snapshotContext())).withName("TransitionTo_" + target);
    }

    public Command cancelAll() {
        // We return a command that parallelizes all stop actions.
        return Commands.parallel(
                intake.stopIntake(),
                shooter.stopShooterCommand(),
                indexer.stopCommand()).withName("SystemCancelAll");
    }

    private Command onExit(SystemState previous) {
        return switch (previous) {
            case INTAKE, DEPLOY, EMPTYING, SHOOT, ALIGNING, UNJAM ->
                cancelAll();
            default -> Commands.none();
        };
    }

    private Command onEntry(SystemState target, Context ctx) {
        return (switch (target) {
            case INTAKE -> intake.startIntakeCommand();
            case DEPLOY -> intake.stowIntakeCommand();
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
                    // 3. Only then, run the indexer to fire the ball
                    shooter.startFeedingCommand(),
                    // Stop when the indexer is empty
                    Commands.waitUntil(() -> debouncer.calculate(indexer.isEmpty())),
                    shooter.stopFeedingCommand());
            case EMPTYING -> Commands.parallel(intake.ejectIntakeCommand(), indexer.reverseCommand());
            case UNJAM -> Commands.parallel(indexer.reverseCommand(), intake.ejectIntakeCommand());
            case RESET -> Commands.parallel(intake.stopIntake(), shooter.stopShooterCommand(), indexer.stopCommand());
            default -> Commands.none();
        }).withName("SystemEntry_" + target);
    }

    // Public interface for manual controls

    public class ManualActions {

        // Intake Controls
        public Command intake() {
            return manualGate(intake.startIntakeCommand());
        }

        public Command intakeStow() {
            return manualGate(intake.stowIntakeCommand());
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

    public SystemState getState() {
        return currentState;
    }

    public BooleanSupplier isInState(SystemState state) {
        return () -> currentState == state;
    }
}