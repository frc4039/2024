// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private Timer timer;

    /** Creates a new AutoShoot. */
    public AutoShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.timer = new Timer();
        addRequirements(shooter, indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shooterPID(ShooterConstants.kShooterRPM);

        if (shooter.getShooterSpeed() >= ShooterConstants.kShooterRPM - 200) {
            indexer.start(IndexerConstants.kIndexerShooterSpeed);
        }

        if (indexer.hasNote() == false) {
            timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shooterSpeedControl(0, 0, 0);
        indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > 0.2;
    }
}
