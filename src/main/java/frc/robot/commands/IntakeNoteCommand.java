// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommand extends Command {
    IntakeSubsystem m_Intake;
    FeederSubsystem m_Feeder;

    /** Creates a new IntakeNote. */
    public IntakeNoteCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_Intake = intake;
        m_Feeder = feeder;
        addRequirements(m_Intake);
        addRequirements(m_Feeder);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!isFinished()) {
            m_Intake.spinIntakeMotor(IntakeConstants.kIntakeSpeed);
            m_Feeder.startFeeder();
        } else {
            end(false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.spinIntakeMotor(0);
        m_Feeder.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Feeder.beamBreakerActivated();
    }
}
