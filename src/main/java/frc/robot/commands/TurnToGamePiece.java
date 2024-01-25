// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToGamePiece extends Command {
  /** Creates a new TurnToGamePiece. */
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier xSpeedSupplier;
  private DoubleSupplier ySpeedSupplier;
  private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP, DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);
  private final NetworkTable nt;

  public TurnToGamePiece(DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    rotationController.setTolerance(Math.PI/360);
    rotationController.enableContinuousInput(0.0,2*Math.PI);
    nt = NetworkTableInstance.getDefault().getTable("photonvision");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(Math.toRadians (driveSubsystem.getHeading()), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cameraTarget = nt.getEntry("targetYaw").getDouble(0);
    rotationController.setGoal(Math.toRadians (driveSubsystem.getHeading() -cameraTarget));
    driveSubsystem.drive(-xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(), rotationController.calculate(Math.toRadians (driveSubsystem.getHeading())),
    true, true);
    SmartDashboard.putNumber("camera-x", nt.getEntry("tx").getDouble(0));
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
