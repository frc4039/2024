// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.utils.Sensors;

public class PivotAngleHPCommand extends Command {
    private PivotAngleSubsystem pivotAngle;
    private double m_angle;

    /** Creates a new PivotAngleHPCommand. */
    public PivotAngleHPCommand(PivotAngleSubsystem pivotAngle, double angle) {
        this.pivotAngle = pivotAngle;
        m_angle = angle;
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
        pivotAngle.setDesiredAngle(m_angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pivotAngle.setDesiredAngle(PivotConstants.kPivotTravelPosition);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Sensors.BeamBreakerIsBroken();
    }
}
