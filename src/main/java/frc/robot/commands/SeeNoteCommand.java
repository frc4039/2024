// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Command Ends when when Vision has not detected a note for kSeeNoteTimeout

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SeeNoteCommand extends Command {

    private DriveSubsystem driveSubsystem;

    double LastDetectedTime;

    public SeeNoteCommand(DriveSubsystem driveSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driveSubsystem = driveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Initial time
        LastDetectedTime = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (driveSubsystem.getNoteDetected())
            LastDetectedTime = Timer.getFPGATimestamp();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - LastDetectedTime) > VisionConstants.kSeeNoteTime);
    }
}