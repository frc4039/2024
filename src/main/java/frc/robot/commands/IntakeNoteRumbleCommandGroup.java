// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteRumbleCommandGroup extends SequentialCommandGroup {
    /** Creates a new IntakeNoteCommandRumble. */
    // public IntakeNoteCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {

    public IntakeNoteRumbleCommandGroup(IntakeSubsystem intake, IndexerSubsystem indexer,
            Joystick driverControler,
            Joystick operatorControler) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelCommandGroup(
                        new IntakeNoteCommand(intake, indexer),
                        new RumbleCommand(driverControler, operatorControler)));
    }
}
