# RobotStateMachine

Overview
--------
RobotStateMachine is the top-level controller that coordinates the TeleopStateMachine and SystemStateMachine. It defines coarse robot modes (DISABLED, AUTO, TELEOP) and provides guarded, idempotent transitions between them.

Responsibilities
- Provide a safe, atomic transition mechanism between top-level robot states
- Run onExit/onEntry hooks and schedule subsystem mode work via Commands
- Expose a Command-based API (`requestState`, `requestStateCommand`) for other code to trigger transitions

Key behavior and implementation notes
- `requestState(RobotState target)` returns a Command (uses `Commands.defer`) so evaluation happens at schedule-time and not during construction.
- `canTransitionTo(Context ctx, RobotState target)` implements transition guards.
- `performTransition` sequences: onExit(previous) -> cancelModeActivities -> update currentState -> onEntry(newState).
- `cancelModeActivities` calls `systemSM.cancelAll()` to stop long-running subsystem activities.

Public API
- Command requestState(RobotState target)
  - Safely request a transition; returns a Command that performs the transition when scheduled.

- Command requestStateCommand(RobotState target)
  - Alias for `requestState` for convenience.

- void cancelAll()
  - Forcibly cancels system-level activities (delegates to SystemStateMachine.cancelAll()).

Notes & TODOs
- onEntry/onExit are kept intentionally small and non-blocking; long-running work should be scheduled as Commands so the transition Command can finish quickly.
- If you have complex autonomous routines, add them to `onEntry(AUTO)` or create a separate AutonomousStateMachine and schedule it here.
