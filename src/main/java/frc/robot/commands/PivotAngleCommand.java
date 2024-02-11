// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotAngleSubsystem;

public class PivotAngleCommand extends Command {
    private PivotAngleSubsystem pivotAngle;

    /** Creates a new PivotAngleCommand. */
    public PivotAngleCommand(PivotAngleSubsystem pivotAngle) {
        this.pivotAngle = pivotAngle;
        addRequirements(pivotAngle);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivotAngle.setDesiredAngle(169);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pivotAngle.setDesiredAngle(225);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
