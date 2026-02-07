package frc.robot.state_machines;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.logging.Level;
import java.util.logging.Logger;
import frc.robot.state_machines.TeleopStateMachine;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SystemState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SystemStateMachine: low-level system behaviors that coordinate subsystems.
 *
 * States:
 * TRAVEL, INTAKE, ALIGNING, SHOOT, INTAKE_AND_SHOOT, RESET, DEPLOY, EMPTYING, UNJAM
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
        INTAKE_AND_SHOOT,
        RESET,
        DEPLOY,
        EMPTYING,
        UNJAM
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

        this.jamDetectedSupplier = Objects.requireNonNull(jamDetectedSupplier);
    }

    // Lightweight Context snapshot for guards
    public static class Context {
        public final SystemState currentState;
        public final boolean jamDetected;

        public Context(SystemState currentState, boolean jamDetected) {
            this.currentState = currentState;
            this.jamDetected = jamDetected;
        }
    }

    private Context snapshotContext() {
        return new Context(
                currentState,
                shooter.isReady(),
                jamDetectedSupplier.getAsBoolean());
    }

    // Public API
    public synchronized RequestResult requestState(SystemState target) {
        if (target == currentState) {
            return RequestResult.ALREADY_IN_STATE;
        }

        Context ctx = snapshotContext();
        RequestResult can = canTransitionTo(ctx, target);
        if (can != RequestResult.ACCEPTED) {
            logger.log(Level.FINE,
                    () -> String.format("System transition %s -> %s rejected: %s (jam=%s)",
                            ctx.currentState, target, can, ctx.jamDetected));
            return can;
        }

        performTransition(target, ctx);
        return RequestResult.ACCEPTED;
    }

    public Command requestStateCommand(SystemState target) {
        return new InstantCommand(() -> requestState(target));
    }

    public RequestResult requestUnjam() {
        return requestState(SystemState.UNJAM);
    }

    // Main transition guard — small switch-based decision tree.
    private RequestResult canTransitionTo(Context ctx, SystemState target) {
        // RESET and UNJAM are allowed from anywhere
        if (target == SystemState.RESET || target == SystemState.UNJAM) {
            return RequestResult.ACCEPTED;
        }
        if (target == SystemState.SHOOT) {
            if (ctx.shooterAtSpeed) {
                return RequestResult.ACCEPTED;
            } else {
                return RequestResult.REJECTED_GUARD;
            }
        }
        if ((intake.getState() != IntakeState.JAM_CLEAR || indexer.getState() != IndexerState.JAM_CLEAR) 
                && target != SystemState.UNJAM) {
            return RequestResult.REJECTED_GUARD;
        }

        switch (ctx.currentState) {
            case TRAVEL:
                if (target == SystemState.INTAKE || target == SystemState.ALIGNING || target == SystemState.DEPLOY
                        || target == SystemState.EMPTYING) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case INTAKE:
                // allow stopping intake (TRAVEL), combining with shoot, aligning, deploying, or
                // emptying
                if (target == SystemState.TRAVEL || target == SystemState.ALIGNING
                        || target == SystemState.DEPLOY || target == SystemState.EMPTYING) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case ALIGNING:
                if (target == SystemState.SHOOT || target == SystemState.TRAVEL
                        || target == SystemState.INTAKE_AND_SHOOT) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case SHOOT:
                if (target == SystemState.TRAVEL || target == SystemState.EMPTYING || target == SystemState.INTAKE) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case INTAKE_AND_SHOOT:
                if (target == SystemState.TRAVEL || target == SystemState.SHOOT || target == SystemState.EMPTYING
                        || target == SystemState.INTAKE) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case RESET:
                // After RESET you usually go to TRAVEL or INTAKE; allow these
                if (target == SystemState.TRAVEL || target == SystemState.INTAKE || target == SystemState.DEPLOY) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case DEPLOY:
                if (target == SystemState.INTAKE || target == SystemState.TRAVEL) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case EMPTYING:
                if (target == SystemState.TRAVEL || target == SystemState.INTAKE) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case UNJAM:
                // After UNJAM go back to TRAVEL
                if (target == SystemState.TRAVEL) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            default:
                return RequestResult.NO_SUCH_TRANSITION;
        }
    }

    // TODO: Finish proofreading code from here down

    // Perform the transition: stop previous activities, set state, run entry
    // actions (non-blocking)
    private void performTransition(SystemState target, Context ctx) {
        SystemState previous = currentState;
        try {
            try {
                onExit(previous);
            } catch (Exception e) {
                logger.log(Level.WARNING, "Exception during SystemState onExit for " + previous, e);
            }

            cancelAll(); // cancel long-running system-level routines (non-blocking)

            currentState = target;
            logger.log(Level.INFO, () -> String.format("System transitioned %s -> %s", previous, target));

            try {
                onEntry(target, snapshotContext());
            } catch (Exception e) {
                logger.log(Level.WARNING, "Exception during SystemState onEntry for " + target, e);
            }
        } finally {
            // optional telemetry publish
        }
    }

    public void cancelAll() {
        // Cancel or stop any running system-level commands/routines.
        try {
            intake.stop();
            shooter.stop();
            indexer.stop();
        } catch (Exception e) {
            logger.log(Level.FINE, "cancelAll subsystem stop failed", e);
        }
    }

    // Fast exit hooks
    private void onExit(SystemState previous) {
        switch (previous) {
            case INTAKE:
            case INTAKE_AND_SHOOT:
            case DEPLOY:
            case EMPTYING:
            case SHOOT:
            case ALIGNING:
                intake.stop();
                shooter.stop();
                indexer.stop();
                break;

            case UNJAM:
                // ensure normal flows are stopped
                indexer.stop();
                intake.stop();
                break;

            default:
                break;
        }
    }

    // Fast entry hooks — non-blocking; schedule longer sequences via Commands
    private void onEntry(SystemState target, Context ctx) {
        switch (target) {
            case TRAVEL:
                break;

            case INTAKE:
                // Deploy and run intake
                intake.deploy();
                intake.start();
                break;

            case DEPLOY:
                // Only deploy intake but don't run rollers
                intake.deploy();
                intake.stop();
                break;

            case ALIGNING:
                // Prepare shooter (spin to target RPM) and run vision/align routine externally
                drivetrain.startVisionAlign();
                break;

            case SHOOT:
                shooter.startShooting();
                break;

            case INTAKE_AND_SHOOT:
                // Run intake and shooter concurrently (feeding will be managed by higher-level
                // routine or command)
                intake.deploy();
                intake.start();
                shooter.startShooting();
                break;

            case EMPTYING:
                intake.eject();
                indexer.reverse();
                break;

            case RESET:
                // Reset routine
                intake.stop();
                intake.stow();

                shooter.stop();
                shooter.stow();

                indexer.stop();
                break;

            case UNJAM:
                // Unjam routine
                indexer.reverse();
                intake.eject();
                break;

            default:
                break;
        }
    }

    public SystemState getState() {
        return currentState;
    }
}