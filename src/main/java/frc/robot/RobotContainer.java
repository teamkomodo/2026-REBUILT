// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DRIVER_XBOX_PORT;
import static frc.robot.Constants.OPERATOR_XBOX_PORT;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.state_machines.SystemStateMachine;
import frc.robot.state_machines.TeleopStateMachine;
import frc.robot.state_machines.RobotStateMachine;
import frc.robot.state_machines.SystemStateMachine.SystemState;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;

public class RobotContainer {

  // Code override
  private final boolean START_IN_MANUAL = false;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_XBOX_PORT);

  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  // Operator override supplier (and underlying value)
  private boolean operatorOverrideValue = false;
  private final BooleanSupplier operatorOverrideSupplier = () -> operatorOverrideValue;

  // State machines
  private final SystemStateMachine systemSM = new SystemStateMachine(intake, shooter, indexer, drivetrain);
  private final TeleopStateMachine teleopSM = new TeleopStateMachine(systemSM, operatorOverrideSupplier,
      START_IN_MANUAL);
  private final RobotStateMachine robotSM = new RobotStateMachine(teleopSM, systemSM, operatorOverrideSupplier);

  // Manual-actions helper from the SystemStateMachine
  private final SystemStateMachine.ManualActions manual = systemSM.getManualActions();

  public RobotContainer() {
    configureBindings();
  }

  // Configure controller button -> command bindings.
  // @formatter:off
  /*
   *   Driver       | Control
   * Joysticks      | Drive
   * X Button       | Zero Gyro
   * Left Bumper    | Toggle Speed Mode
   * 
   *   Operator     | Control
   * Left Bumper    | Operator Override Toggle
   * X Button       | Enter Manual Control (needs Operator Override)
   * Right Trigger  | Intake (System INTAKE)
   * Left Trigger   | Stow (System STOW)
   * POV Down       | Eject (System EMPTYING)
   * POV Left       | Teleop STEAL (needs Operator Override)
   * POV Right      | Teleop SCORE (needs Operator Override)
   * A Button       | SHOOT (System SHOOT)
   * Y Button       | --- (only used in manual mode)
   * POV Up         | --- (only used in manual mode)
   * Right Bumper   | Start/Stop Feeding (pressed/unpressed) (System SHOOT)
   * B Button       | Feed Once (Shoot once)
   */
  /*
   * Manual Control  | System Action
   *   Operator      | Control
   * Left Bumper     | Operator Override Toggle (needs Operator Override)
   * X Button        | Exit Manual Control
   * Right Trigger   | Manual Intake (Duty Cycle)
   * Left Trigger    | Manual Intake Stow
   * POV Down        | Manual Eject (Indexer/Intake)
   * A Button        | Manual Shooter (Short)
   * Y Button        | Manual Shooter (Long)
   * POV Up          | Manual Shooter (Pass)
   * Right Bumper    | Manual Indexer Feed Start/Stop (pressed/unpressed)
   * B Button        | Manual Indexer Feed Once
   */
  // @formatter:on
  private void configureBindings() {
    // Driver controls
    Trigger driverX = driverController.x();
    Trigger driverLB = driverController.leftBumper();
    // Trigger driverRB = driverController.rightBumper(); // Currently unused

    driverX.onTrue(drivetrain.zeroGyroCommand());
    driverLB.onTrue(drivetrain.disableSpeedModeCommand());
    driverLB.onFalse(drivetrain.enableSpeedModeCommand());
    // driverRB reserved for align/auto actions if implemented
    // driverRB.onTrue(/* some align command */);

    // Operator controls (manual gates are provided by
    // SystemStateMachine.ManualActions)
    Trigger operatorRT = operatorController.rightTrigger();
    Trigger operatorLT = operatorController.leftTrigger();
    Trigger operatorA = operatorController.a();
    Trigger operatorB = operatorController.b();
    Trigger operatorX = operatorController.x();
    Trigger operatorY = operatorController.y();
    Trigger operatorRB = operatorController.rightBumper();
    Trigger operatorLB = operatorController.leftBumper();
    Trigger operatorPOVDown = operatorController.povDown();
    Trigger operatorPOVUp = operatorController.povUp();
    Trigger operatorPOVLeft = operatorController.povLeft();
    Trigger operatorPOVRight = operatorController.povRight();

    // OPERATOR OVERRIDE
    operatorLB
        .onTrue(Commands.runOnce(() -> operatorOverrideValue = true))
        .onFalse(Commands.runOnce(() -> operatorOverrideValue = false));

    // Enter Manual Mode
    operatorX.onTrue(
        Commands.defer(() -> {
          TeleopState targetState = teleopSM.isInState(TeleopState.MANUAL) ? TeleopState.SCORE : TeleopState.MANUAL;
          Command requestCommand = teleopSM.requestState(targetState);
          return requestCommand;
        },
            Set.of(teleopSM)));

    // Intake
    // Call both the non-manual (state request) and the manual-gated action.
    // Non-manual requests come first so the guard/transition is evaluated before
    // the manual command (the manual command is gated to MANUAL state).
    operatorRT.onTrue(Commands.parallel(systemSM.requestState(SystemState.INTAKE), manual.intake()));
    operatorLT.onTrue(Commands.parallel(systemSM.requestState(SystemState.STOW), manual.intakeStow()));
    operatorPOVDown.onTrue(Commands.parallel(systemSM.requestState(SystemState.EMPTYING), manual.intakeEject()));

    // Teleop quick switches (non-manual): POV left/right pick STEAL/SCORE modes
    operatorPOVLeft.onTrue(teleopSM.requestState(TeleopState.STEAL));
    operatorPOVRight.onTrue(teleopSM.requestState(TeleopState.SCORE));

    // Shooter
    // Map face buttons to both manual shot commands and a guarded request to enter
    // SHOOT.
    // Shooter: request SHOOT + teleop SCORE (so the system and teleop modes align)
    operatorA.onTrue(Commands.parallel(systemSM.requestState(SystemState.SHOOT), manual.shootShort()));
    operatorY.onTrue(Commands.parallel(Commands.none(), manual.shootLong()));
    operatorPOVUp.onTrue(Commands.parallel(Commands.none(), manual.shootPass()));
    // Start feeding should normally be part of SHOOT; request SHOOT too.
    // operatorRB
    //     .onTrue(Commands.parallel(systemSM.requestState(SystemState.SHOOT), manual.startFeeding()))
    //     .onFalse(Commands.parallel(shooter.stopFeedingCommand(), manual.stopFeeding()));
    // // Shoot once
    // operatorB.onTrue(Commands.parallel(systemSM.requestState(SystemState.SHOOT), manual.feedOnce()));

    // Default drivetrain command (joystick driving)
    drivetrain.setDefaultCommand(
        drivetrain.joystickDriveCommand(
            () -> (driverController.getLeftY()), // left Y -> robot +X
            () -> (driverController.getLeftX()), // left X -> robot +Y
            () -> (driverController.getRightX() / 1.6) // rotation scaled
        ));
  }

  public Command getAutonomousCommand() {
    // If you later add an auto chooser, return selected command here.
    return null;
  }

  public void startTeleop() {
    // Request the top-level robot state machine to enter TELEOP and start the
    // teleop timeline (non-blocking; these return Commands and are scheduled).
    CommandScheduler.getInstance().schedule(robotSM.requestState(RobotStateMachine.RobotState.TELEOP));
    CommandScheduler.getInstance().schedule(teleopSM.teleopMasterCommand());
  }

  public void enterDisabledMode() {
    // Ensure the RobotStateMachine transitions to DISABLED and put teleop into safe
    // state.
    CommandScheduler.getInstance().schedule(robotSM.requestState(RobotStateMachine.RobotState.DISABLED));
    teleopSM.enterDisabled();
  }
}
