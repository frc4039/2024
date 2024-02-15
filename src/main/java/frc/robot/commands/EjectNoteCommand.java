// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectNoteCommand extends Command {
    IntakeSubsystem m_Intake;
    IndexerSubsystem m_Indexer;

    /** Creates a new IntakeNote. */
    public EjectNoteCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_Intake = intake;
        m_Indexer = indexer;
        addRequirements(m_Intake);
        addRequirements(m_Indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Intake.spinIntakeMotor(-IntakeConstants.kIntakeSpeedMotor);
        m_Indexer.start(-IndexerConstants.kIndexerIntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.spinIntakeMotor(0);
        m_Indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
