// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPreSpinShooter extends Command {
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;

    /** Creates a new PreSpinShooter. */
    public AutoPreSpinShooter(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (indexer.hasNote()) {
            shooter.shooterPID(ShooterConstants.kShooterRPM);
        } else {
            shooter.shooterSpeedControl(0, 0, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
