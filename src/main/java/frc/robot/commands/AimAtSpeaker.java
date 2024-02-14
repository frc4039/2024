// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtSpeaker extends Command {
    /** Creates a new AimAtSpeaker. */
    private DriveSubsystem driveSubsystem;
    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);

    public AimAtSpeaker(DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        rotationController.setTolerance(Math.PI / 360);
        rotationController.enableContinuousInput(0.0, 2 * Math.PI);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotationController.reset(driveSubsystem.getPose().getRotation().getRadians(), 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double targetangle = driveSubsystem.getTranslationToGoal().getAngle().getRadians();
        rotationController.setGoal(targetangle);
        driveSubsystem.drive(-xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(),
                rotationController.calculate(driveSubsystem.getPose().getRotation().getRadians()),
                true, true);
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
