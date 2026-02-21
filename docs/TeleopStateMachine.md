# TeleopStateMachine

Overview
--------
TeleopStateMachine implements fine-grained teleop modes and an automatic match-timeline-driven mode switcher (SCORE vs STEAL shifts). It coordinates with `SystemStateMachine` to enable intake/shoot sequences according to match time and the game-specific message.

Responsibilities
- Manage teleop sub-modes: IDLE, STEAL, SCORE, MANUAL, RESET
- Provide `teleopMasterCommand()` which implements the timed match behavior (initial SCORE, determine active hub, alternate STEAL/SCORE shifts, etc.)
- Provide guarded `requestState(...)` API for switching teleop modes

Key implementation details
- `teleopMasterCommand()` uses a `ParallelDeadlineGroup` to wait up to 10 seconds for game-specific message availability and then determines which hub is active.
- `isOurHubInactiveFirst()` parses driver-station game data and the alliance to decide initial match behavior. (You confirmed this logic is correct for 2026.)
- Transition guard logic is implemented in `TeleopState.canTransitionTo(...)` and supports operator override as well as automatic transitions.

Public API
- Command requestState(TeleopState target[, boolean isAutomaticTransition])
  - Returns a deferred Command that validates and performs transitions at schedule time.

- Command teleopMasterCommand()
  - High-level timeline Command that implements the typical timed teleop shifts; schedule this at teleop start if you want automated behavior.

Notes & TODOs
- The `teleopMasterCommand()` contains the comment "FIXME: Make sure other commands will not cancel this" — if you plan to rely on it, ensure no other code schedules conflicting commands that claim the same subsystem requirements. You may want to make it hold a lightweight subsystem requirement (or design with `CommandGroups` that avoid interfering commands).
- `receivedGameData` and the `canDetermineActiveHubSupplier` supplier are used to capture and debounce the DriverStation message; this is a reasonable approach but validate behavior in simulation and on-field.
