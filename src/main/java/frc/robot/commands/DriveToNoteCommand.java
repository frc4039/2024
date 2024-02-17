// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class DriveToNoteCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);

    /** Creates a new DriveToNoteCommand. */
    public DriveToNoteCommand(DriveSubsystem driveSubsystem, IndexerSubsystem indexerSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(driveSubsystem);
        rotationController.setTolerance(Math.PI / 360);
        rotationController.enableContinuousInput(0.0, 2 * Math.PI);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotationController.reset(Math.toRadians(driveSubsystem.getHeading()), 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rotationController.setGoal(Math.toRadians(driveSubsystem.getHeading() - driveSubsystem.getNoteAngle()));
        driveSubsystem.drive(DriveConstants.kDriveToNoteXSpeed, DriveConstants.kDriveToNoteYSpeed,
                rotationController.calculate(Math.toRadians(driveSubsystem.getHeading())),
                false, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return indexerSubsystem.hasNote();
    }
}
