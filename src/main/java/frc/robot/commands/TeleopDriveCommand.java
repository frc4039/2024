// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command takes operator control from the driver and controls the swerve
 * motors with that control signal.
 */
public class TeleopDriveCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private DoubleSupplier rotSpeedSupplier;
    private Double rotationAngle;
    private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);

    /** Creates a new TeleopDrive. */
    public TeleopDriveCommand(DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSpeedSupplier,
            Double rotationAngle) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.rotationAngle = rotationAngle;
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
        if (rotationAngle != -1.0) {
            rotationController.setGoal(rotationAngle);
            driveSubsystem.drive(-xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(),
                    rotationController.calculate(Math.toRadians(driveSubsystem.getHeading())),
                    true, true);
        } else {
            driveSubsystem.drive(-xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(),
                    -rotSpeedSupplier.getAsDouble(),
                    true, true);
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
        return false;
    }
}
