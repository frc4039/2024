// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PreSpinShooter extends Command {
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private Supplier<ScoringState> scoringState;

    /** Creates a new PreSpinShooter. */
    public PreSpinShooter(ShooterSubsystem shooter, IndexerSubsystem indexer, Supplier<ScoringState> scoringState) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.indexer = indexer;
        this.scoringState = scoringState;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (indexer.hasNote()
                && (scoringState.get() == ScoringState.HIGH || scoringState.get() == ScoringState.PodiumShoot
                        || scoringState.get() == ScoringState.SubwooferShoot)) {
            shooter.shooterPID(ShooterConstants.kShooterRPM * 0.7);
        } else if (indexer.hasNote()
                && scoringState.get() == ScoringState.SHUTTLE) {
                     shooter.shooterPID(ShooterConstants.kShuttleShootRPM);
        }else {
            shooter.shooterSpeedControl(0, 0, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shooterSpeedControl(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
