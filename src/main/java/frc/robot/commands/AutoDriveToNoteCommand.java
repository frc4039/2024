// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class AutoDriveToNoteCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double xSpeed;
    private boolean stopDriving;
    private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);
    private Optional<Alliance> allianceColour;

    /** Creates a new DriveToNoteCommand. */
    public AutoDriveToNoteCommand(DriveSubsystem driveSubsystem, IndexerSubsystem indexerSubsystem,
            IntakeSubsystem intakeSubsystem, double xSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xSpeed = xSpeed;
        addRequirements(driveSubsystem);
        rotationController.setTolerance(Math.PI / 360);
        rotationController.enableContinuousInput(0.0, 2 * Math.PI);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotationController.reset(Math.toRadians(driveSubsystem.getHeading()), 0);
        stopDriving = false;
        allianceColour = DriverStation.getAlliance();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // If Stop Driving is enabled, do nothing

        if (!stopDriving) {
            // stop driving if Crossing Centerline or motor Current indicates note in intake
            if (intakeSubsystem.getOutputCurrent() > IntakeConstants.IntakeNoteCurrentThreshold
                    || (this.allianceColour.isPresent() && this.allianceColour.get() == Alliance.Blue
                            && driveSubsystem.getPoseXValue() > 8.25 + AutoConstants.CenterLineCrossThreshold)
                    || (this.allianceColour.isPresent() && this.allianceColour.get() == Alliance.Red
                            && driveSubsystem.getPoseXValue() < 8.25 - AutoConstants.CenterLineCrossThreshold)) {
                driveSubsystem.drive(0, 0, 0, false, false);
                stopDriving = true;
            } else {
                // Drive toward note
                rotationController.setGoal(Math.toRadians(driveSubsystem.getHeading() - driveSubsystem.getNoteAngle()));
                driveSubsystem.drive(xSpeed, DriveConstants.kDriveToNoteYSpeed,
                        rotationController.calculate(Math.toRadians(driveSubsystem.getHeading())),
                        false, true);
            }
        }
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
