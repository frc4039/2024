// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeIndexShootCommandGroup extends ParallelDeadlineGroup {
    /** Creates a new IntakeIndexShootCommandGroup. */
    public IntakeIndexShootCommandGroup(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
            IntakeSubsystem intakeSubsystem, BlinkinSubsystem blinkinSubsystem, Joystick driverController,
            Joystick operatorController) {
        // Add the deadline command in the super() call. Add other commands using
        // addCommands().
        super(new AutoShootCommand(shooterSubsystem, indexerSubsystem));

        addCommands(
                new IntakeNoteRumbleCommandGroup(intakeSubsystem, indexerSubsystem, blinkinSubsystem, driverController,
                        operatorController),
                new IndexerCommand(indexerSubsystem, shooterSubsystem, IndexerConstants.kIndexerShooterSpeed));
    }
}