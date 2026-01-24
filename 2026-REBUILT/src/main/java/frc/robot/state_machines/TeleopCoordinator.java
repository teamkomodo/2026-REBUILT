

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machines.TeleopStateMachine;
import edu.wpi.first.wpilibj.DriverStation;

public class TeleopCoordinator extends SubsystemBase {
    private final TeleopStateMachine teleopSm;
    private final BooleanSupplier teleopActive;
    private final BooleanSupplier operatorOverride;

    private final long transitionMs = 10_000L;
    private final long allianceShiftMs = 25_1000L;
    private boolean autoToggleEnabled = true;

    private long teleopStartedMs = -1;
    private long lastToggleMs = -1;
    private long lastManualRequestMs = -1;
    private final long manualPauseMs = 10_000L; // pause auto after manual change

    public TeleopCoordinator(TeleopStateMachine teleopSm, BooleanSupplier teleopActive) {
        this.teleopSm = teleopSm;
        this.teleopActive = teleopActive;
        this.operatorOverride = teleopSm.calibrationCompleteSupplier();
    }

    public void onTeleopStart(long nowMs) {
        teleopStartedMs = nowMs;
        lastToggleMs = -1;
        lastManualRequestMs = -1;
    }

    public void onTeleopEnd() {
        teleopStartedMs = -1;
    }

    public void notifyManualRequest(long nowMs) {
        lastManualRequestMs = nowMs;
    }

    // Call this from Robot.teleopPeriodic()
    public void periodic(long nowMs) {
        if (!teleopActive.getAsBoolean()) {
            return;
        }
        if (teleopStartedMs < 0) {
            onTeleopStart(nowMs);
        }
        if (!autoToggleEnabled) return;
        if (operatorOverride.getAsBoolean()) return;
        if (nowMs - teleopStartedMs < transitionMs) return;

        TeleopStateMachine.State s = teleopSm.getCurrentState();
        if (s == TeleopStateMachine.State.STEAL || s == TeleopStateMachine.State.SCORE) {
            long elapsed = (lastToggleMs < 0) ? (nowMs - teleopStartedMs) : (nowMs - lastToggleMs);
            if (nowMs - lastManualRequestMs < manualPauseMs) return;
            if (elapsed >= allianceShiftMs) {
                TeleopStateMachine.State next = (s == TeleopStateMachine.State.STEAL) ? TeleopStateMachine.State.SCORE : TeleopStateMachine.State.STEAL;
                TeleopStateMachine.RequestResult r = teleopSm.requestState(next);
                if (r == TeleopStateMachine.RequestResult.ACCEPTED) {
                    lastToggleMs = nowMs;
                    // publish telemetry/log
                } else {
                    // optional: record last attempt time to avoid spam
                }
            }
        }

        // Example: auto-end calibration
        if (s == TeleopStateMachine.State.CALIBRATE) {
            if (calibrationCompleteSupplier.getAsBoolean() || nowMs - teleopStartedMs > calibrationTimeout) {
                teleopSm.requestState(TeleopStateMachine.State.IDLE);
            }
        }
    }
}