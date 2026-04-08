package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartFeedingCommand extends DynamicCommand {

    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public StartFeedingCommand(
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);

    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
                shooterSubsystem.startFeedingCommand(),
                intakeSubsystem.updateIntakeDutyCycle(INTAKE_INTAKE_SPEED));
    }
}