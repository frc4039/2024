// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkinSubsystem;

public class BlinkinCommand extends Command {
    private BlinkinSubsystem m_BlinkinSubsystem;
    private double m_ColourCode;

    /** Creates a new BlinkinCommand. */
    public BlinkinCommand(BlinkinSubsystem blinkinSubsystem, double colourCode) {
        m_BlinkinSubsystem = blinkinSubsystem;
        m_ColourCode = colourCode;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_BlinkinSubsystem.SetColour(m_ColourCode);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
