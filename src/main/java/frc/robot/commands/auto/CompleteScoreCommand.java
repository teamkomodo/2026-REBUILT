package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CompleteScoreCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public CompleteScoreCommand(
                IntakeSubsystem intakeSubsystem,
                ShooterSubsystem shooterSubsystem,
                IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(shooterSubsystem);
        addRequirements(indexerSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            shooterSubsystem.longShotCommand(),
            new WaitCommand(3.0),
            indexerSubsystem.startCommand(),
            shooterSubsystem.startFeedingCommand(),
            new WaitCommand(5.0),
            shooterSubsystem.stopShooterCommand(),
            shooterSubsystem.stopFeedingCommand(),
            indexerSubsystem.stopIndexerCommand()
        );
            
    }
}