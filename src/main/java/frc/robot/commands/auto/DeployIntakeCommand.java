package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(
                IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            intakeSubsystem.deployIntake()
        );
            
    }
}