// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveToNoteParallelRaceGroup extends ParallelRaceGroup {
    /** Drive to Note with Vision in Auto. */
    public AutoDriveToNoteParallelRaceGroup(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
            DriveSubsystem driveSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new SeeNoteCommand(driveSubsystem),
                new IntakeNoteCommand(intakeSubsystem, indexerSubsystem),
                new AutoDriveToNoteCommand(driveSubsystem, indexerSubsystem, intakeSubsystem,
                        DriveConstants.kAutoDriveToNoteXSpeed)
                        .withTimeout(DriveConstants.kAutoDriveToNoteTime));

    }
}
