package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.INTAKE_FEED_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopFeedCommand extends DynamicCommand {

    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public StopFeedCommand(
            ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);

    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
                shooterSubsystem.stopFeedingCommand(),
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.updateIntakeDutyCycle(INTAKE_FEED_SPEED));
    }
}