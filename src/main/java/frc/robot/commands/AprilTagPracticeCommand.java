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
import frc.robot.LimelightHelpers;

public class AprilTagPracticeCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double xSpeed;
    private boolean stopDriving;
    private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);
    private Optional<Alliance> allianceColour;

    /** Creates a new DriveToNoteCommand. */
    public AprilTagPracticeCommand(DriveSubsystem driveSubsystem, double xSpeed) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

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
        LimelightHelpers.setPipelineIndex("", 0);
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        boolean hasTarget = LimelightHelpers.getTV("");

        if (tx > 0.5) {
            driveSubsystem.drive(0, 0.07, 0, false, true);
        }
        if (tx < -0.5) {
            driveSubsystem.drive(0, -0.07, 0, false, true);
        }
        if (tx < 0.5 && tx > -0.5) {
            driveSubsystem.drive(0, 0, 0, false, true);
            System.out.println("centered");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return indexerSubsystem.hasNote();
    }

    /*
     * public static double getTx() {
     * LimelightHelpers.setPipelineIndex("", 0);
     * double tx = LimelightHelpers.getTX("");
     * return tx;
     * }
     * 
     * public static double getTy() {
     * LimelightHelpers.setPipelineIndex("", 0);
     * double ty = LimelightHelpers.getTY("");
     * return ty;
     * }
     * 
     * public static boolean getTv() {
     * LimelightHelpers.setPipelineIndex("", 0);
     * boolean hasTarget = LimelightHelpers.getTV("");
     * return hasTarget;
     * }
     */
}
