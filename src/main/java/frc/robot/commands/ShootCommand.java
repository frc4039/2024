// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private DoubleSupplier ShooterValue;
  private ShooterSubsystem shooter;
  /** Creates a new ShootCommand. */
  public ShootCommand(DoubleSupplier ShooterValue, ShooterSubsystem shooter) {
    this.ShooterValue = ShooterValue;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("output", ShooterValue.getAsDouble());
    shooter.shooterSpeedControl(ShooterValue.getAsDouble(),ShooterValue.getAsDouble(), 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterSpeedControl(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
