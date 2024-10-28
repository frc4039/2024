// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class AdjustClimbAnalogRightTriggerCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    // private double percentOutput;
    // private DoubleSupplier bias;

    public AdjustClimbAnalogRightTriggerCommand(ClimberSubsystem climberSubsystem, DoubleSupplier bias) {
        this.climberSubsystem = climberSubsystem;
        // this.bias = bias;
        addRequirements(climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // climberSubsystem.setClimbPercentOutput(percentOutput, bias.getAsDouble());
        climberSubsystem.setClimbSpeed(0.25, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
