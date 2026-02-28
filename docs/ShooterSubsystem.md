# ShooterSubsystem

Overview
--------
ShooterSubsystem controls the flywheel (two motors: master + follower) and a feeder motor that advances balls into the flywheel. It exposes simple APIs for spinning the flywheel, starting/stopping the feeder, and a single-shot feed operation.

Responsibilities
- Command flywheel velocity and duty-cycle
- Command feeder motor for continuous or single-ball feeding
- Publish telemetry for flywheel & feeder
- Provide convenience Command factories used by the SystemStateMachine

Key fields and hardware
- shooterMotorRight/shooterMotorLeft: SparkMax controllers for flywheel (right is master)
- shooterMotorRightController: closed-loop controller for flywheel
- shooterMotorRightRelativeEncoder: flywheel encoder (velocity in RPM depending on config)
- feederMotor: SparkMax for feeding balls into flywheel
- feederController & feederEncoder: closed-loop controller + encoder for feeder

Public API (important methods)
- Command updateShooterSpeed(double desiredSpeed)
  - Sets flywheel velocity setpoint (internal desiredSpeed updated and set once).

- void setShooterDutyCycle(double dutyCycle)
  - Sets flywheel duty-cycle (open-loop control).

- Command startShootingCommand()
  - Convenience to set shooter to a lookup-table speed for a placeholder distance (used by SystemStateMachine entry points).

- void startFeeding() / stopFeeding()
  - Start/stop continuous feeder at configured duty cycle.

- void updateFeederDutyCycle(double dutycycle)
  - Update desiredFeederSpeed and set feeder duty-cycle immediately.

- void feedOnce()
  - Advances feeder by `SHOOTER_FEEDER_ROTATIONS_PER_BALL` using position control (calls feederController.setSetpoint(..., kPosition)).

- boolean isAtTargetSpeed()
  - Returns true if flywheel velocity within `MAX_SHOOTER_SPEED_TOLERANCE` of desiredSpeed (uses absolute tolerance).

Telemetry
- `shooter-speed`, `shooter-rpm`, `shooter-desired-speed`
- `feeder-speed`, `feeder-rpm`, `feeder-desired-speed`

Notes & TODOs
- PID gains for both flywheel and feeder are currently placeholders and marked FIXME — tune on robot.
- `feedOnce()` uses closed-loop position control. If you do not want to tune position PID, consider replacing `feedOnce()` with a duty-cycle + encoder delta wait approach (example command factory is recommended in docs).
- Confirm encoder units and whether `getPosition()` returns rotations on your SparkMax setup (the current constant `SHOOTER_FEEDER_ROTATIONS_PER_BALL` assumes rotations).

Suggested improvements
- Add NetworkTable keys for feeder-state (running/idle) or scheduled command name for better debugging.
- Add a `feedOnceCommand()` implemented as a deferred Sequence that runs feeder duty-cycle and waits for encoder delta if you prefer not to tune position PID.
