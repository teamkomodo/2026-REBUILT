// package main.java.frc.robot.state_machines; // VSCode unhappy if uncommented for Elijah

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Mid-level coordination state machine.
 * States: TRAVEL, INTAKE, ALIGNING, SHOOT, INTAKE_AND_SHOOT, UNJAM
 * Coordinates subsystem API calls (intake/indexer/shooter/drivetrain).
 */
public class SystemStateMachine {
    public enum SystemState { TRAVEL, INTAKE, ALIGNING, SHOOT, INTAKE_AND_SHOOT, UNJAM }

    private SystemState current = SystemState.TRAVEL;

    // Subsystems (interfaces) to call into
    private final frc.robot.subsystems.IntakeSubsystem intake;
    private final frc.robot.subsystems.ShooterSubsystem shooter;
    private final frc.robot.subsystems.IndexerSubsystem indexer;
    private final frc.robot.subsystems.DrivetrainSubsystem drivetrain;

    public SystemStateMachine(
            frc.robot.subsystems.IntakeSubsystem intake,
            frc.robot.subsystems.ShooterSubsystem shooter,
            frc.robot.subsystems.IndexerSubsystem indexer,
            frc.robot.subsystems.DrivetrainSubsystem drivetrain) {
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        publish();
    }

    public SystemState getState() {
        return current;
    }

    public void periodic() {
        // Minimal coordinator: run behavior for current state
        switch (current) {
            case TRAVEL:
                // ensure subsystems are neutral/stowed
                intake.stow();
                indexer.stop();
                shooter.stopShooting();
                // drivetrain remains in drive mode (operator-controlled)
                break;

            case INTAKE:
                intake.deploy();
                intake.start();
                indexer.start();
                break;

            case ALIGNING:
                // coordinate: prepare shooter, stop feeding if necessary
                shooter.aim(); // default aim (or no-arg)
                shooter.start(); // spinup
                indexer.stop();
                // drivetrain align logic runs inside drivetrain subsystem or elsewhere
                break;

            case SHOOT:
                // assume shooter is spun up / aimed — strategy or operator ensures that before calling shoot
                shooter.startShooting(); // start feeding/shoot sequence
                indexer.start(); // feed indexer to shooter
                break;

            case INTAKE_AND_SHOOT:
                // coordinated: intake while shooting when ready; keep logic minimal here
                intake.deploy();
                intake.start();
                shooter.start();
                // indexer may alternate between AGITATE and feed depending on design
                break;

            case UNJAM:
                // run jam-clear sequences - subsystems handle the behavior
                intake.stop();
                intake.getState(); // keep for logging
                intake.stow(); // or intensity depending on your routine
                // instruct indexer and intake to run JAM_CLEAR states if implemented
                indexer.reverse();
                break;
        }

        publish();
    }

    private void publish() {
        SmartDashboard.putString("SystemState", current.name());
        SmartDashboard.putString("IntakeState", intake.getState().name());
        SmartDashboard.putString("ShooterState", shooter.getState().name());
        SmartDashboard.putString("IndexerState", indexer.getState().name());
        SmartDashboard.putString("DrivetrainState", drivetrain.getState().name());
    }

    // External request to change system state (Strategy calls this)
    public void requestState(SystemState requested) {
        // You will add transition gates here (and use operator override in TeleopStateMachine)
        this.current = requested;
        publish();
    }

    public void resetToIdle() {
        this.current = SystemState.TRAVEL;
        publish();
    }
}