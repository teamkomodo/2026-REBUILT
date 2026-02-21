# IntakeSubsystem

Overview
--------
The IntakeSubsystem manages two related subsystems: the intake rollers that pick up balls and the hinge that positions the intake. It publishes telemetry for both the intake and hinge and provides Command factories used by the SystemStateMachine.

Responsibilities
- Run intake rollers (master + follower)
- Control hinge position with an absolute encoder and closed-loop controller
- Provide higher-level Commands (start intake, feed, eject, stow)
- Publish telemetry for debugging

Key fields and hardware
- intakeMotorRight/intakeMotorLeft: SparkMax motor controllers (right is master)
- intakeMotorRightController: closed-loop controller for intake roller
- hingeMotorRight/hingeMotorLeft: SparkMax for hinge (right is master)
- hingeAbsoluteEncoder: absolute encoder for hinge position (preferred for hold)

Public API (important methods)
- Command startIntakeCommand()
  - Deploys the hinge to intake position and runs the intake rollers.

- Command feedIntakeCommand()
  - Puts the intake into feeding posture (hinge feed position + intake feed speed).

- Command ejectIntakeCommand()
  - Runs an eject sequence to clear jams or expel balls.

- Command stowIntakeCommand()
  - Stows the hinge and stops the intake rollers.

- void setIntakeDutyCycle(double dutyCycle)
  - Low-level duty-cycle control for the intake master motor.

- void setHingePosition(double position)
  - Command hinge to a closed-loop position setpoint.

Telemetry
- `intake-speed`, `intake-rpm`, `intake-desired-speed`
- `hinge-speed`, `hinge-rpm`, `hinge-desired-position`, `hinge-absolute-position`

Notes & TODOs
- Several motor IDs and PID gains are placeholders and are marked FIXME in `Constants` and the class.
- `hingeAbsoluteEncoder` is used for hold/position, which is recommended to avoid drift.
- Verify the follower/inversion setup: intakeMotorLeft follows the right motor and is inverted; hinge follower behavior should match mechanical wiring.

Design rationale
- The intake provides small Command factories (Sequential/Parallel groups) so callers can compose intake/hinge behaviors easily from the SystemStateMachine.
