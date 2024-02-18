// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotAngleSubsystem;

public class PivotToTravelCommand extends InstantCommand {
    private PivotAngleSubsystem pivot;

    /** Creates a new PivotToTravelCommand. */
    public PivotToTravelCommand(PivotAngleSubsystem pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.pivot.setDesiredAngle(PivotConstants.kPivotTravelPosition);
    }
}
