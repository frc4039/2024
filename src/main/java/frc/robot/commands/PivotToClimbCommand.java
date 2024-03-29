// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.Constants.PivotConstants;

public class PivotToClimbCommand extends Command {
    private PivotAngleSubsystem pivotAngle;
    private DriveSubsystem Drive;
    private double m_angle;
    private double StartPosX;
    private double StartPosY;
    private double Distance = 0.0;
    private boolean Driving = false;
    private double driveDistance = 0.4;

    /** Creates a new PivotToClimbCommand. */
    public PivotToClimbCommand(PivotAngleSubsystem pivotAngle, DriveSubsystem Drive, double angle) {
        this.pivotAngle = pivotAngle;
        this.Drive = Drive;
        m_angle = angle;
        addRequirements(pivotAngle);
        addRequirements(Drive);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        StartPosX = Drive.getPose().getX();
        StartPosY = Drive.getPose().getY();
        pivotAngle.setDesiredAngle(211);
        Driving = false;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (pivotAngle.getPitch() <= 215) {
            if (Driving == false) {
                Drive.drive(-.05, 0.0, 0.0, false, true);
                Driving = true;
            }
            double x = Drive.getPose().getX() - StartPosX;
            double y = Drive.getPose().getY() - StartPosY;
            double newAngle;
            Distance = Math.sqrt(x * x + y * y);
            newAngle = m_angle + (211 - m_angle) * (1 - Distance / driveDistance);
            if (newAngle > PivotConstants.kPivotTravelPosition)
                newAngle = PivotConstants.kPivotTravelPosition;
            if (newAngle < m_angle)
                newAngle = m_angle;
            pivotAngle.setDesiredAngle(newAngle);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Drive.drive(0.0, 0.0, 0.0, false, true);
        pivotAngle.setDesiredAngle(m_angle);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Distance >= driveDistance ? true : false;
    }
}