// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ActivateTrapSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ActivateTrapCommand extends Command {
    /** Creates a new ActivateTrapCommand. */
    private boolean ActivateTrap;
    private ActivateTrapSubsystem TrapSubsystem;
    private Timer ResetTime;

    public ActivateTrapCommand(ActivateTrapSubsystem TrapSubsystem, boolean ActivateTrap) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.ActivateTrap = ActivateTrap;
        this.TrapSubsystem = TrapSubsystem;
        addRequirements(TrapSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (ActivateTrap) {
            TrapSubsystem.Release();
        } else {
            TrapSubsystem.Reset();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
