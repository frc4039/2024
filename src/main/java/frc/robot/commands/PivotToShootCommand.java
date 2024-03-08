// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;

public class PivotToShootCommand extends Command {
    private PivotAngleSubsystem pivot;
    private DriveSubsystem drive;
    private InterpolatingDoubleTreeMap angleEstimator = new InterpolatingDoubleTreeMap();

    /** Creates a new PivotToShoot. */
    public PivotToShootCommand(PivotAngleSubsystem pivot, DriveSubsystem drive) {
        this.pivot = pivot;
        this.drive = drive;
        addRequirements(pivot);
        angleEstimator.put(PivotConstants.kPivotDistanceFar, PivotConstants.kPivotAngleFar);
        angleEstimator.put(PivotConstants.kPivotDistanceClose, PivotConstants.kPivotAngleClose);
        angleEstimator.put(PivotConstants.kPivotDistanceMedium, PivotConstants.kPivotAngleMedium);
        angleEstimator.put(PivotConstants.kPivotDistance4, PivotConstants.kPivotAngle4);
        angleEstimator.put(PivotConstants.kPivotDistance5, PivotConstants.kPivotAngle5);
        angleEstimator.put(PivotConstants.kPivotDistance6, PivotConstants.kPivotAngle6);
        angleEstimator.put(PivotConstants.kPivotDistance7, PivotConstants.kPivotAngle7);
        angleEstimator.put(PivotConstants.kPivotDistance8, PivotConstants.kPivotAngle8);
        angleEstimator.put(PivotConstants.kPivotDistance9, PivotConstants.kPivotAngle9);
    }

    // Helpers.isBabycakes() ? 3.0 : 242.0

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivot.setDesiredAngle(angleEstimator.get(drive.getTranslationToGoal().getNorm()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pivot.setDesiredAngle(PivotConstants.kPivotTravelPosition);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
