package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartFeedingCommand extends DynamicCommand {

    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public StartFeedingCommand(
                ShooterSubsystem shooterSubsystem,
                IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(indexerSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            indexerSubsystem.startCommand(),
            shooterSubsystem.startFeedingCommand()
        );
            
    }
}