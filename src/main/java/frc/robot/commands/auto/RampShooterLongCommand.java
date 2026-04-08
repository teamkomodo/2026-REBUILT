package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class RampShooterLongCommand extends DynamicCommand {

    private final ShooterSubsystem shooterSubsystem;

    public RampShooterLongCommand(
                ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            shooterSubsystem.longShotCommandAuto()
        );
            
    }
}