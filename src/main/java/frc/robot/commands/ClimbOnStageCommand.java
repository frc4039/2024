// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbOnStageCommand extends Command {

    private ClimberSubsystem m_Climber;
    private double climbSpeed;
    private DoubleSupplier bias;

    public ClimbOnStageCommand(ClimberSubsystem climber, double climbSpeed, DoubleSupplier bias) {
        m_Climber = climber;
        this.climbSpeed = climbSpeed;
        this.bias = bias;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (climbSpeed > 0.0) {
            m_Climber.setClimbSpeed(climbSpeed + bias.getAsDouble() * ClimberConstants.kClimberBiasLimit,
                    climbSpeed - bias.getAsDouble() * ClimberConstants.kClimberBiasLimit);
        } else {
            if (bias.getAsDouble() < 0) {
                m_Climber.setClimbSpeed(climbSpeed * bias.getAsDouble(), 0);
            } else {
                m_Climber.setClimbSpeed(0, climbSpeed * bias.getAsDouble());
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Climber.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
