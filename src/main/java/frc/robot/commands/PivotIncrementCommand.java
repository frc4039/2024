// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotAngleSubsystem;

public class PivotIncrementCommand extends Command {

	private PivotAngleSubsystem pivotAngleSubsystem;
	private double incrementSpeed;

	/**
	 * Creates a new PivotIncrement Command.
	 * @param pivotAngleSubsystem The pivot subsystem.
	 * @param incrementSpeed The speed at which the pivot should move, in degrees per second
	 */
	public PivotIncrementCommand(PivotAngleSubsystem pivotAngleSubsystem, double incrementSpeed) {
		this.incrementSpeed = incrementSpeed / 50.0;
		this.pivotAngleSubsystem = pivotAngleSubsystem;

		addRequirements(pivotAngleSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double desiredAngle = pivotAngleSubsystem.getPitch() + incrementSpeed;

		// Make sure we don't try to move to an angle the robot can't move to
		if(desiredAngle < PivotConstants.kPivotTravelPosition && desiredAngle > PivotConstants.kPivotAmpPosition){
			pivotAngleSubsystem.setDesiredAngle(desiredAngle);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(pivotAngleSubsystem.getPitch() - PivotConstants.kPivotTrapPosition) < 1;
	}
}
