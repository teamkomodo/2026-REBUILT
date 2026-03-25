package frc.robot.commands.auto;

import static frc.robot.Constants.INTAKE_AUTO_DUTYCYCLE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartFeedingCommand extends DynamicCommand {

    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public StartFeedingCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(indexerSubsystem);
        addRequirements(intakeSubsystem);

    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
                shooterSubsystem.startFeedingCommand(),
                intakeSubsystem.updateIntakeDutyCycle(INTAKE_AUTO_DUTYCYCLE));
    }
}