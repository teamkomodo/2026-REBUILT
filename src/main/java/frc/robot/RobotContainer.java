// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.state_machines.SystemStateMachine;
import frc.robot.state_machines.TeleopStateMachine;
import frc.robot.commands.auto.CompleteScoreCommand;
import frc.robot.state_machines.RobotStateMachine;
import frc.robot.state_machines.RobotStateMachine.RobotState;
import frc.robot.state_machines.SystemStateMachine.SystemState;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;

public class RobotContainer {

  // Code override
  private final boolean START_IN_MANUAL = true;

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
  private final TeleopStateMachine teleopSM = new TeleopStateMachine(systemSM, operatorOverrideSupplier);
  private final RobotStateMachine robotSM = new RobotStateMachine(teleopSM, systemSM, operatorOverrideSupplier);

  // Manual-actions helper from the SystemStateMachine
  private final SystemStateMachine.ManualActions manual = systemSM.getManualActions();

  private Command teleopMasterCommand; // the master command that runs the teleop timeline (scheduled in startTeleop)

  public RobotContainer() {
    configureBindings();

    registerNamedCommands();
  }

  // Configure controller button -> command bindings.
  // @formatter:off
  /*
   *   Driver       | Control
   * --------------------------------------------
   * Joysticks      | Drive
   * X Button       | Zero Gyro
   * Left Bumper    | Toggle Speed Mode
   */


   /*
   * Manual Control  | System Action
   * Operator        | Control
   * ---------------------------------------------
   * Right Bumper    | Manual Intake (Deploy & Start)
   * Left Bumper     | Manual Shooter Feed (Start/Stop)
   * Right Trigger   | Manual Shoot Short (Ramp Up)
   * Left Trigger    | Manual Short Long (Ramp Up)
   * A Button        | Manual Stop All
   * X Button        | Manual Shooter Feed (Once)
   * B Button        | 
   * Y Button        | 
   * POV Up          | Manual Intake Stow
   * POV Down        | Manual Eject
   * POV Right       | 
   * POV Left        | 
   */



  // @formatter:on
  private void configureBindings() {
    SmartDashboard.putNumber("LEFT_STEER_OFFSET", FRONT_LEFT_STEER_OFFSET);
    // Driver controls
    Trigger driverX = driverController.x();
    Trigger driverLB = driverController.leftBumper();
    Trigger driverRB = driverController.rightBumper(); // Currently unused

    driverX.onTrue(drivetrain.zeroGyroCommand());
    driverLB.onTrue(drivetrain.disableSpeedModeCommand());
    driverLB.onFalse(drivetrain.enableSpeedModeCommand());
    driverRB.onTrue(Commands.runOnce(() -> System.out.println("=========  RECONFIGURE PID"))
        .andThen(shooter.reconfigureRobotTuningCommand()));
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
    // operatorLB
    //     .onTrue(Commands.runOnce(() -> operatorOverrideValue = true))
    //     .onFalse(Commands.runOnce(() -> operatorOverrideValue = false));

    // Enter Manual Mode
    // Toggle mode is commented out right now
    // operatorX.onTrue(
    // Commands.defer(() -> {
    // System.out.println("==== Attempting to toggle MANUAL state. Current state is
    // manual?: "
    // + teleopSM.isInState(TeleopState.MANUAL));
    // TeleopState targetState = teleopSM.isInState(TeleopState.MANUAL) ?
    // TeleopState.SCORE : TeleopState.MANUAL;
    // Command requestCommand = teleopSM.requestState(targetState);
    // return requestCommand;
    // }, Set.of(teleopSM)));
    //operatorX.onTrue(teleopSM.requestState(TeleopState.MANUAL));

    // Intake
    // Call both the non-manual (state request) and the manual-gated action.
    // Non-manual requests come first so the guard/transition is evaluated before
    // the manual command (the manual command is gated to MANUAL state).
    operatorRB.onTrue(manual.intake());
    operatorPOVUp.onTrue(manual.intakeStow());
    operatorPOVDown.onTrue(manual.eject());

    // Teleop quick switches (non-manual): POV left/right pick STEAL/SCORE modes

    // Shooter
    // Map face buttons to both manual shot commands and a guarded request to enter
    // SHOOT.
    // Shooter: request SHOOT + teleop SCORE (so the system and teleop modes align)
    operatorRT.onTrue(manual.shootShort());
    operatorLT.onTrue(manual.shootLong());
    operatorA.onTrue(manual.reset());
    // operatorPOVUp.onTrue(Commands.parallel(systemSM.requestState(SystemState.SHOOT),
    // manual.shootPass()));
    // Start feeding should normally be part of SHOOT; request SHOOT too.
    operatorLB
        .onTrue(manual.startFeeding())
        .onFalse(manual.stopFeeding());
    // // Shoot once

    operatorX.onTrue(Commands.parallel(systemSM.requestState(SystemState.SHOOT), manual.feedOnce()));

    // Default drivetrain command (joystick driving)
    drivetrain.setDefaultCommand(
        drivetrain.joystickDriveCommand(
            () -> (-driverController.getLeftX()), // left Y -> robot +X
            () -> (driverController.getLeftY()), // left X -> robot +Y
            () -> (driverController.getRightX() / 1.6) // rotation scaled
        ));
  }

  public Command getAutonomousCommand() {
    // If you later add an auto chooser, return selected command here.
    return null;
    //return AutoBuilder.buildAuto("Shoot");
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Reset", new CompleteScoreCommand(intake, shooter, indexer));
  }

  public void startTeleop() {
    // Request the top-level robot state machine to enter TELEOP and start the
    // teleop timeline (non-blocking; these return Commands and are scheduled).
    CommandScheduler.getInstance().schedule(robotSM.requestState(RobotState.TELEOP));
    // No master command; it is not helping
    // CommandScheduler.getInstance().schedule(teleopMasterCommand =
    // teleopSM.teleopMasterCommand());
    if (START_IN_MANUAL) {
      CommandScheduler.getInstance().schedule(
        Commands.sequence(
          Commands.runOnce(() -> { operatorOverrideValue = true; }),
          teleopSM.requestState(TeleopState.MANUAL),
          Commands.runOnce(() -> { operatorOverrideValue = false; })
        )
      );
    }
  }

  public void enterDisabledMode() {
    // Ensure the RobotStateMachine transitions to DISABLED and put teleop into safe
    // state.
    if (teleopMasterCommand != null) {
      teleopMasterCommand.cancel();
      teleopMasterCommand = null;
    }
    CommandScheduler.getInstance().schedule(
        Commands.sequence(
            robotSM.requestState(RobotState.DISABLED),
            teleopSM.enterDisabled()));
  }
}
// COMMANDS FOR AUTO (NOT USED FOR NOW!)

/**
   *   Operator     | Control
   * ---------------------------------------------
   *   Left Bumper    | Operator Override Toggle
   * X Button       | Enter Manual Control (needs Operator Override)
   * Right Trigger  | Intake (System INTAKE)
   * Left Trigger   | Stow (System STOW)
   * POV Down       | Eject (System EMPTYING)
   * A Button       | SHOOT (System SHOOT)
   * Y Button       | Reset Robot
   * POV Up         | Long SHOOT
   * POV Left       | Teleop STEAL (needs Operator Override)
   * POV Right      | Teleop SCORE (needs Operator Override)
   * Right Bumper   | Start/Stop Feeding (pressed/unpressed) (System SHOOT)
   * B Button       | Feed Once (Shoot once)
   * ---------------------------------------------
   * 
   */