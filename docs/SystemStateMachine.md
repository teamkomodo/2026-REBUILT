# SystemStateMachine

Overview
--------
SystemStateMachine coordinates low-level system behaviors that span multiple subsystems (Intake, Shooter, Indexer, Drivetrain). It models states such as TRAVEL, INTAKE, ALIGNING, SHOOT, EMPTYING and provides guarded transitions with concise onEntry/onExit work.

Responsibilities
- Define allowed system-level states and valid transitions
- Schedule concrete subsystem Commands in `onEntry` to implement each state's behavior (for example, SHOOT sequences the shooter and feeder)
- Provide utility commands and triggers for automatic state changes (e.g., when indexer beam-breaks fire)

Key implementation details
- `configureTriggers()` sets up `Trigger`s that automatically change system state when indexer beam-breaks indicate 'FULL' or 'EMPTY' while in certain states.
- `requestState(SystemState target)` returns a deferred Command that validates the transition before performing it.
- `onEntry(SHOOT)` sequences the following behavior:
  1. Start shooter spinup
  2. Wait until `shooter.isAtTargetSpeed()`
  3. Start feeder
  4. Wait until indexer is empty (debounced)
  5. Stop feeder

Manual interface
- `ManualActions` inner class exposes manually-invokable commands for operator control, gated to `SystemState.MANUAL` via `manualGate()`.

Telemetry & safety
- `cancelAll()` returns a Command that stops intake, shooter, and indexer activity — useful for emergency stops or state exits.
- The class uses a `Debouncer` to avoid spurious transitions caused by noisy beam-breaks.

Notes & TODOs
- Ensure `Debouncer` duration (`BEAMBREAK_DEBOUNCE_DURATION`) is tuned to the physical ball bounce characteristics.
- The SHOOT sequence assumes `shooter` and `indexer` Command factories behave idempotently and correctly; unit testing those Command sequences is recommended.
