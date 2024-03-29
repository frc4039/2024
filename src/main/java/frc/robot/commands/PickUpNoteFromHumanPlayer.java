// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpNoteFromHumanPlayer extends SequentialCommandGroup {
    /** Creates a new PickUpNoteFromHumanPlayer. */
    public PickUpNoteFromHumanPlayer(DriveSubsystem drive, PivotAngleSubsystem pivotAngle, ShooterSubsystem shooter,
            IndexerSubsystem indexer) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new HumanPlayerIntakeCommand(shooter, indexer),
                new PivotToShootCommand(pivotAngle, drive));
    }
}
