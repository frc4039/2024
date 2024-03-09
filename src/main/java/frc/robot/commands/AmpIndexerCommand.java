// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpIndexerCommand extends Command {
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private PivotAngleSubsystem pivotAngleSubsystem;
    private double targetSpeed;
    private double desiredAngle;

    /** Creates a new IndexerCommand. */
    public AmpIndexerCommand(PivotAngleSubsystem pivot, IndexerSubsystem indexer,
            ShooterSubsystem shooter, double targetSpeed,
            double desiredAngle) {
        this.pivotAngleSubsystem = pivot;
        this.indexer = indexer;
        this.shooter = shooter;
        this.targetSpeed = targetSpeed;
        this.desiredAngle = desiredAngle;

        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(shooter.getShooterSpeed()) >= targetSpeed
                && Math.abs(pivotAngleSubsystem.getPitch()) < (desiredAngle + 4)) {
            indexer.start(IndexerConstants.kIndexerShooterSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
