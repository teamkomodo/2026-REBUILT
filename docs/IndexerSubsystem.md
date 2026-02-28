# IndexerSubsystem

Overview
--------
IndexerSubsystem is responsible for staging balls between the intake and the shooter. It controls a single motorized indexer and reads two beam-break sensors to determine whether the indexer is full or has a ball ready to shoot.

Responsibilities
- Run the indexer motor in forward/reverse or duty-cycle modes
- Provide simple Commands for start/stop/reverse behavior used by the SystemStateMachine
- Publish telemetry about motor output and sensor state

Key fields and hardware
- indexerMotor: SparkMax motor controller
- indexerMotorController: closed-loop controller used for velocity/duty control
- indexerMotorRelativeEncoder: encoder on indexer motor (rotations/velocity)
- beamBreakIsFull: DigitalInput for 'indexer is full' beam-break
- beamBreakIsReady: DigitalInput for 'piece ready' beam-break

Public API (important methods)
- Command stopCommand()
  - Stops the indexer and sets state to IDLE.

- Command reverseCommand()
  - Runs the indexer in reverse (used for unjamming / ejecting).

- Command startCommand()
  - Starts the indexer (AGITATING) at the configured speed.

- void updateSpeed(double desiredSpeed)
  - Updates the internal desiredSpeed and commands the motor (velocity control).

- void setSpeed(double speed)
  - Directly sets velocity setpoint via the closed-loop controller.

- void setDutyCycle(double dutyCycle)
  - Sets duty-cycle output using the closed-loop controller (kDutyCycle mode).

- boolean isIndexerFull()
  - Returns beamBreakIsFull.get() (true when sensor triggered).

- boolean isPieceReady()
  - Returns beamBreakIsReady.get() (true when a piece is staged at ready position).

- boolean isEmpty()
  - Returns true when neither ready nor full is triggered.

Telemetry
- `indexer-speed` publishes applied motor output
- `indexer-state` publishes current IndexerState
- `indexer-full-sensor` and `indexer-ready-sensor` publish beam-break states

Notes & TODOs
- PID gains are currently set to placeholder values (see `indexerPidGains`) and must be tuned.
- Beam-break reads are wrapped in try/catch to avoid NT exceptions on bad hardware; this is defensive but fine for now.
- Confirm encoder units (rotations vs ticks) match expectations if you later use position-based index operations.

Design rationale
- The indexer provides minimal, composable commands (runOnce wrappers) so the SystemStateMachine can safely sequence indexer work.
