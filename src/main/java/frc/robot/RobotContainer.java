// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.state_machines.SystemStateMachine;
import frc.robot.state_machines.TeleopStateMachine;
import frc.robot.commands.auto.CompleteScoreCommand;
import frc.robot.commands.auto.DeployIntakeCommand;
import frc.robot.commands.auto.RampShooterLongCommand;
import frc.robot.commands.auto.StartFeedingCommand;
import frc.robot.commands.auto.StopFeedCommand;
import frc.robot.state_machines.RobotStateMachine;
import frc.robot.state_machines.RobotStateMachine.RobotState;
import frc.robot.state_machines.TeleopStateMachine.TeleopState;
import frc.robot.util.PeriodTimer;
import frc.robot.util.XboxController;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  // Code override
  private final boolean START_IN_MANUAL = true;

  // Controllers
  private final XboxController driver = new XboxController(DRIVER_XBOX_PORT);
  private final XboxController operator = new XboxController(OPERATOR_XBOX_PORT);
  private final XboxController coach = new XboxController(2);

  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(field);
  private final PoseEstimationSubsystem poseEstimationSubsystem = new PoseEstimationSubsystem(drivetrain);
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(poseEstimationSubsystem);

  // Operator override supplier (and underlying value)
  private boolean operatorOverrideValue = false;
  private final BooleanSupplier operatorOverrideSupplier = () -> operatorOverrideValue;

  // State machines
  private final SystemStateMachine systemSM = new SystemStateMachine(intake, shooter, indexer, drivetrain);
  private final TeleopStateMachine teleopSM = new TeleopStateMachine(systemSM, operatorOverrideSupplier);
  private final RobotStateMachine robotSM = new RobotStateMachine(teleopSM, systemSM, operatorOverrideSupplier);

  // Timer
  private final Timer teleopTimer = new Timer();

  // Manual-actions helper from the SystemStateMachine
  private final SystemStateMachine.ManualActions manual = systemSM.getManualActions();
  private final PeriodTimer periodTimer = new PeriodTimer(teleopTimer);
  private double timeLeft = 0;

  public static final NetworkTable timeNT = NetworkTableInstance.getDefault().getTable("time");
  private final DoublePublisher timePublisher = timeNT.getDoubleTopic("time").publish();

  private Command teleopMasterCommand; // the master command that runs the teleop timeline (scheduled in startTeleop)

  public RobotContainer() {
    configureBindings();

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // Configure controller button -> command bindings.
  // @formatter:off
  /*
   *   Driver       | Control
   * --------------------------------------------
   * Joysticks      | Drive
   * X Button       | Zero Gyro
   * Left Bumper    | Toggle Speed Mode
   * B Button       | Toggle auto lock rotation to (face) our alliance hub
   */


   /*
   * Manual Control  | System Action
   * Operator        | Control
   * ---------------------------------------------
   * Right Bumper    | Manual Intake (Deploy & Start)
   * Left Bumper     | Manual Shooter Feed (Start/Stop)
   * Right Trigger   | Manual Shooter Autodistance Toggle On/Off // XXX: Replaces shooter short
   * Right Trigger   | Manual Shoot Short (Ramp Up) // XXX: Replaces shooter long
   * A Button        | Manual Stop All
   * X Button        | 
   * B Button        | 
   * Y Button        | 
   * POV Up          | Manual Intake Stow
   * POV Down        | Manual Eject
   * POV Right       | 
   * POV Left        | 
   */



  // @formatter:on
  private void configureBindings() {

    // Driver controls

    driver.x.onTrue(drivetrain.zeroGyroCommand());
    driver.lb.onTrue(drivetrain.disableSpeedModeCommand());
    driver.lb.onFalse(drivetrain.enableSpeedModeCommand());
    driver.b.onTrue(drivetrain.toggleAutoAlignCommand());
    driver.y.onTrue(shooter.runShooterToShuffleboardRPM());
    driver.a.onTrue(shooter.toggleAutoDistanceCommand());

    // Default drivetrain command (joystick driving)
    drivetrain.setDefaultCommand(
        drivetrain.joystickDriveCommand(
            () -> (-driver.getLeftJoystickX()), // left Y -> robot +X
            () -> (driver.getLeftJoystickY()), // left X -> robot +Y
            () -> (driver.getRightJoystickX() / 1.6) // rotation scaled
        ));

    operator.rb.onTrue(Commands.parallel(manual.intake(),
        Commands.runOnce(() -> operator.rumbleSmooth(0.2))));
    operator.povUp.onTrue(Commands.parallel(manual.intakeDeploy(),
        Commands.runOnce(() -> operator.rumbleSmooth(0.2))));
    operator.povDown.onTrue(manual.eject());

    // Teleop quick switches (non-manual): POV left/right pick STEAL/SCORE modes

    // Shooter
    // Map face buttons to both manual shot commands and a guarded request to enter
    // SHOOT.
    // Shooter: request SHOOT + teleop SCORE (so the system and teleop modes align)
    operator.rt.onTrue(manual.shootShort());
    operator.lt.onTrue(manual.shootLong());

    operator.b.onTrue(Commands.parallel(manual.stopIntake(),
        Commands.runOnce(() -> operator.stopSmoothRumble())));

    operator.a.onTrue(manual.stopShooter());

    operator.lb
        .onTrue(manual.startFeeding())
        .onFalse(manual.stopFeeding());

    // Tuning
    operator.leftStick
        .onTrue(Commands.runOnce(() -> System.out.println("=========  RECONFIGURE PID"))
            .andThen(shooter.reconfigureRobotTuningCommand()));
  }

  public Command getAutonomousCommand() {
    // If you later add an auto chooser, return selected command here.
    if (autoChooser != null) {
      return autoChooser.getSelected();
    }
    return null;
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", new CompleteScoreCommand(shooter, indexer));
    NamedCommands.registerCommand("Ramp Shooter Long", new RampShooterLongCommand(shooter));
    NamedCommands.registerCommand("Deploy Intake", new DeployIntakeCommand(intake));
    NamedCommands.registerCommand("Intake", intake.startIntakeAutoCommand());
    NamedCommands.registerCommand("Feed All", new StartFeedingCommand(shooter, intake));
    NamedCommands.registerCommand("Stop", new StopFeedCommand(shooter, intake));
    NamedCommands.registerCommand("Reset Odom", new WaitCommand(0.1));
  }

  public void startTeleop() {
    resetTimer();
    // Request the top-level robot state machine to enter TELEOP and start the
    // teleop timeline (non-blocking; these return Commands and are scheduled).
    CommandScheduler.getInstance().schedule(robotSM.requestState(RobotState.TELEOP));
    // No master command; it is not helping\
    if (START_IN_MANUAL) {
      CommandScheduler.getInstance().schedule(
          Commands.sequence(
              Commands.runOnce(() -> {
                operatorOverrideValue = true;
              }),
              teleopSM.requestState(TeleopState.MANUAL),
              Commands.runOnce(() -> {
                operatorOverrideValue = false;
              })));
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

  // @N/A 20s autonomous period
  // @0s 10s transition shift
  // @10s 25s loser shift
  // @35s 25s winner shift
  // @60s 25s loser shift
  // @85s 25s winner shift
  // @110s 30s endgame shift
  // @140s 0s game end

  public void setRumbles() {

    if (periodTimer.getController5SecRumble()) {
      driver.rumbleBoth(0.05);
      coach.rumbleBoth(0.05);
    } else if (periodTimer.getController10SecRumble()) {
      driver.rumbleBoth(1.0);
      coach.rumbleBoth(1.0);
    } else {
      driver.stopRoughRumble();
      driver.stopSmoothRumble();
      coach.stopRoughRumble();
      coach.stopSmoothRumble();
    }

    double rpm = Math.abs(shooter.getShooterMotorRPM());
    if (rpm > (4275)) {
      operator.rumbleRough(1.0);
    } else {
      operator.stopRoughRumble();
    }
  }

  public void resetTimer() {
    teleopTimer.restart();
  }

  public double getTime() {
    return teleopTimer.get();
  }

  public void periodic() {
    setRumbles();
    timeLeft = periodTimer.getPeriodTimer();
    // System.out.println(timeLeft);
    timePublisher.set(timeLeft);
  }

}

// COMMANDS FOR AUTO (NOT USED FOR NOW!)

/**
 * Operator | Control
 * ---------------------------------------------
 * Left Bumper | Operator Override Toggle
 * X Button | Enter Manual Control (needs Operator Override)
 * Right Trigger | Intake (System INTAKE)
 * Left Trigger | Stow (System STOW)
 * POV Down | Eject (System EMPTYING)
 * A Button | SHOOT (System SHOOT)
 * Y Button | Reset Robot
 * POV Up | Long SHOOT
 * POV Left | Teleop STEAL (needs Operator Override)
 * POV Right | Teleop SCORE (needs Operator Override)
 * Right Bumper | Start/Stop Feeding (pressed/unpressed) (System SHOOT)
 * B Button | Feed Once (Shoot once)
 * ---------------------------------------------
 * 
 */