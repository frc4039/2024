// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbOnStageCommand extends Command {

    private ClimberSubsystem m_Climber;
    private boolean m_ClimbUP;

    public ClimbOnStageCommand(ClimberSubsystem climber, boolean ClimbUP) {
        m_Climber = climber;
        m_ClimbUP = ClimbUP;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_ClimbUP) {
            m_Climber.ClimbOnStage(ClimberConstants.kClimberMotorSpeed);
        } else {
            m_Climber.ClimbOnStage(-1 * ClimberConstants.kClimberMotorSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Climber.StopClimbing();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
