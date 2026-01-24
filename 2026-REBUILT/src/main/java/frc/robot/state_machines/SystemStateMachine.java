// package main.java.frc.robot.state_machines; // VSCode unhappy if uncommented (for Elijah)

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.logging.Level;
import java.util.logging.Logger;
import frc.robot.state_machines.TeleopStateMachine;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.state_machines.SystemStateMachine.IntakeSubsystem;
import frc.robot.state_machines.SystemStateMachine.IndexerSubsystem;
import frc.robot.state_machines.SystemStateMachine.ShooterSubsystem;
import frc.robot.state_machines.SystemStateMachine.DrivetrainSubsystem;

/**
 * SystemStateMachine: low-level system behaviors that coordinate subsystems.
 *
 * States:
 * TRAVEL, INTAKE, ALIGNING, SHOOT, INTAKE_AND_SHOOT, CALIBRATE_INTAKE,
 * CALIBRATE_SHOOTER, RESET, DEPLOY, EMPTYING, UNJAM
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
        TRAVEL,
        INTAKE,
        ALIGNING,
        SHOOT,
        INTAKE_AND_SHOOT,
        CALIBRATE_INTAKE,
        CALIBRATE_SHOOTER,
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

    // Suppliers for calibration/jam statuses. Must be cheap to call.
    private final BooleanSupplier intakeCalibratedSupplier;
    private final BooleanSupplier shooterCalibratedSupplier;
    private final BooleanSupplier jamDetectedSupplier; // optional, used for richer logging/guards

    private volatile SystemState currentState = SystemState.TRAVEL;
    private volatile long stateEnteredTimestampMs = System.currentTimeMillis();

    public SystemStateMachine(
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            DrivetrainSubsystem drivetrain,
            BooleanSupplier intakeCalibratedSupplier,
            BooleanSupplier shooterCalibratedSupplier,
            BooleanSupplier jamDetectedSupplier) {

        this.intake = Objects.requireNonNull(intake);
        this.shooter = Objects.requireNonNull(shooter);
        this.indexer = Objects.requireNonNull(indexer);
        this.drivetrain = Objects.requireNonNull(drivetrain);

        this.intakeCalibratedSupplier = Objects.requireNonNull(intakeCalibratedSupplier);
        this.shooterCalibratedSupplier = Objects.requireNonNull(shooterCalibratedSupplier);
        this.jamDetectedSupplier = Objects.requireNonNull(jamDetectedSupplier);
    }

    // Lightweight Context snapshot for guards
    public static class Context {
        public final SystemState currentState;
        public final long stateEnteredTimestampMs;
        public final boolean intakeCalibrated;
        public final boolean shooterCalibrated;
        public final boolean jamDetected;

        public Context(SystemState currentState, long stateEnteredTimestampMs, boolean intakeCalibrated,
                boolean shooterCalibrated, boolean jamDetected) {
            this.currentState = currentState;
            this.stateEnteredTimestampMs = stateEnteredTimestampMs;
            this.intakeCalibrated = intakeCalibrated;
            this.shooterCalibrated = shooterCalibrated;
            this.jamDetected = jamDetected;
        }
    }

    private Context snapshotContext() {
        return new Context(
            currentState,
            stateEnteredTimestampMs,
            intakeCalibratedSupplier.getAsBoolean(),
            shooterCalibratedSupplier.getAsBoolean(),
            jamDetectedSupplier.getAsBoolean()
        );
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
                    () -> String.format("System transition %s -> %s rejected: %s (intakeCal=%s, shootCal=%s, jam=%s)",
                            ctx.currentState, target, can, ctx.intakeCalibrated, ctx.shooterCalibrated,
                            ctx.jamDetected));
            return can;
        }

        performTransition(target, ctx);
        return RequestResult.ACCEPTED;
    }

    public Command requestStateCommand(SystemState target) {
        return new InstantCommand(() -> requestState(target));
    }

    // Convenience commands used by higher-level code
    public RequestResult requestCalibrateIntake() {
        return requestState(SystemState.CALIBRATE_INTAKE);
    }

    public RequestResult requestCalibrateShooter() {
        return requestState(SystemState.CALIBRATE_SHOOTER);
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
                if (target == SystemState.SHOOT || target == SystemState.TRAVEL || target == SystemState.INTAKE_AND_SHOOT) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case SHOOT:
                if (target == SystemState.TRAVEL || target == SystemState.EMPTYING || target == SystemState.INTAKE) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case INTAKE_AND_SHOOT:
                if (target == SystemState.TRAVEL || target == SystemState.SHOOT || target == SystemState.EMPTYING || target == SystemState.INTAKE) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case CALIBRATE_INTAKE:
                // After intake calibration we typically go to TRAVEL; require intake
                // calibration flag to be true
                if (target == SystemState.TRAVEL) {
                    return ctx.intakeCalibrated ? RequestResult.ACCEPTED : RequestResult.REJECTED_GUARD;
                }
                if (target == SystemState.RESET) {
                    return RequestResult.ACCEPTED;
                }
                return RequestResult.NO_SUCH_TRANSITION;

            case CALIBRATE_SHOOTER:
                if (target == SystemState.TRAVEL) {
                    return ctx.shooterCalibrated ? RequestResult.ACCEPTED : RequestResult.REJECTED_GUARD;
                }
                if (target == SystemState.RESET) {
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
            stateEnteredTimestampMs = System.currentTimeMillis();
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
        // If you maintain handles to active Commands, cancel them here. For now, stop
        // actuators that shouldn't keep running.
        // Example:
        // intake.stop(); shooter.stopSpin(); indexer.stop();
        try {
            intake.stop();
            shooter.stopSpin();
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
                // Ensure intake motors are in a safe state
                try {
                    intake.stop();
                    if (previous == SystemState.DEPLOY) {
                        // optionally stow after deploy exit if desired
                        // intake.stow();
                    }
                } catch (Exception e) {
                    logger.log(Level.FINE, "Intake cleanup failed onExit", e);
                }
                break;

            case SHOOT:
            case ALIGNING:
                try {
                    shooter.stopSpin();
                    indexer.stop();
                } catch (Exception e) {
                    logger.log(Level.FINE, "Shooter/indexer cleanup failed onExit", e);
                }
                break;

            case CALIBRATE_INTAKE:
            case CALIBRATE_SHOOTER:
                // calibration routines are responsible for leaving hardware in a safe state;
                // ensure motors are stopped
                try {
                    intake.stop();
                    shooter.stopSpin();
                    indexer.stop();
                } catch (Exception e) {
                    logger.log(Level.FINE, "Calibration onExit cleanup failed", e);
                }
                break;

            case UNJAM:
                // ensure normal flows are stopped
                try {
                    indexer.stop();
                    intake.stop();
                } catch (Exception e) {
                    logger.log(Level.FINE, "Unjam onExit cleanup failed", e);
                }
                break;

            default:
                break;
        }
    }

    // Fast entry hooks — non-blocking; schedule longer sequences via Commands
    private void onEntry(SystemState target, Context ctx) {
        switch (target) {
            case TRAVEL:
                // Make intake safe for travel
                intake.stow();
                intake.stop();
                shooter.stopSpin();
                indexer.stop();
                break;

            case INTAKE:
                // Deploy and run intake
                intake.deploy();
                intake.start(); // start intake rollers
                // indexer may be running slowly to hold balls; leave that to a scheduled
                // routine
                break;

            case DEPLOY:
                // Only deploy intake but don't run rollers
                intake.deploy();
                intake.stop();
                break;

            case ALIGNING:
                // Prepare shooter (spin to target RPM) and run vision/align routine externally
                shooter.spinToTarget(); // non-blocking start
                indexer.stop(); // do not feed until ready
                break;

            case SHOOT:
                // Start shooting sequence: ensure shooter at RPM, then indexer feeds; schedule
                // as a command
                shooter.spinToTarget();
                // Note: do not drive indexer here synchronously; schedule a shooting routine
                // command that monitors RPM
                break;

            case INTAKE_AND_SHOOT:
                // Run intake and shooter concurrently (feeding will be managed by higher-level
                // routine or command)
                intake.deploy();
                intake.start();
                shooter.spinToTarget();
                break;

            case EMPTYING:
                // Eject all game pieces slowly; use indexer/ejector logic
                intake.stow();
                intake.eject(); // run intake in reverse or dedicated eject routine
                indexer.eject();
                break;

            case CALIBRATE_INTAKE:
                // Start intake calibration routine; it should asynchronously set
                // intakeCalibratedSupplier / underlying flag
                // Ensure motors are in a known state first
                intake.stow();
                intake.stop();
                startIntakeCalibrationProcedure();
                break;

            case CALIBRATE_SHOOTER:
                // Start shooter calibration routine
                shooter.stop();
                startShooterCalibrationProcedure();
                break;

            case RESET:
                // Start a reset procedure that moves actuators to known positions; schedule as
                // an async command
                startResetProcedure();
                break;

            case UNJAM:
                // Run an unjam routine: reverse indexer/intake to clear jams then return to
                // TRAVEL.
                startUnjamProcedure();
                break;

            default:
                break;
        }
    }

    // Async procedure starters (placeholders) — implement these with Commands or
    // scheduled tasks
    private void startIntakeCalibrationProcedure() {
        // Kick off non-blocking intake calibration; when done ensure
        // intakeCalibratedSupplier becomes true.
    }

    private void startShooterCalibrationProcedure() {
        // Kick off non-blocking shooter calibration; when done ensure
        // shooterCalibratedSupplier becomes true.
    }

    private void startResetProcedure() {
        // Non-blocking reset routine that homes/zeros subsystems, then higher-level
        // code may requestState(TRAVEL).
    }

    private void startUnjamProcedure() {
        // Non-blocking unjam routine. Example steps:
        // - reverse indexer briefly, then run intake ejection, then stop and leave in
        // TRAVEL-safe config.
        indexer.reversePulse();
        intake.eject();
        // higher-level code should call requestState(TRAVEL) when unjam routine
        // finishes (or the routine can request transition).
    }

    // Accessors / utilities
    public SystemState getCurrentState() {
        return currentState;
    }

    public long getStateEnteredTimestampMs() {
        return stateEnteredTimestampMs;
    }

    // --- Minimal subsystem interfaces used by SystemStateMachine ---
    public interface IntakeSubsystem {
        void deploy();

        void stow();

        void start();

        void stop();

        void lock();

        void eject();

        boolean isStowed();

        IntakeState getState();
    }

    public enum IntakeState {
        STOW, DEPLOY, INTAKE, LOCK, EJECT, JAM_CLEAR
    }

    public interface ShooterSubsystem {
        void spinToTarget(); // non-blocking start

        void stopSpin(); // stop spin

        void stop(); // alias/backup

        boolean isAtTargetRPM();
    }

    public interface IndexerSubsystem {
        void start();

        void stop();

        void eject();

        void reversePulse();

        boolean isJammed();
    }
}