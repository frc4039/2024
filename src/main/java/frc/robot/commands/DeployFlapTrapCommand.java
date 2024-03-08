// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class DeployFlapTrapCommand extends Command {
    private ClimberSubsystem m_Climber;

    /** Creates a new DeployFlapTrapCommand. */
    public DeployFlapTrapCommand(ClimberSubsystem climber) {
        m_Climber = climber;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Climber.deployFlapTrap();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Climber.unDeployFlapTrap();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
