package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IndexerSubsystem extends SubsystemBase {


    private IndexerState indexerState = IndexerState.IDLE;

    public static enum IndexerState {
        IDLE,
        PIECE_READY,
        AGITATING,
        INDEXER_FULL,
        REVERSE
    };

    public IndexerSubsystem() {
    }
    
    // Blank methods (easy fix for motor removal)
    public IndexerState getState() {
        return indexerState;
    }

    public Command stopIndexerCommand() {
        return Commands.none();
    }

    public Command reverseCommand() {
        return Commands.none();
    }

    public Command startCommand() {
        return Commands.none();
    }

    public void updateSpeed(double desiredSpeed) {
    }

    public void setSpeed(double speed) {
    }

    public void setDutyCycle(double dutyCycle) {
    }

    public void teleopInit() {
    }

    @Override
    public void periodic() {
    }

    public void configureMotors() {
    }

    public void updateTelemetry() {
    }
}
