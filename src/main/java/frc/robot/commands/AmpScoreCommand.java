// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScoreCommand extends SequentialCommandGroup {
    /** Creates a new ScoreYellowHigh. */
    public AmpScoreCommand(PivotAngleSubsystem pivotAngleSubsystem, ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem) {
        addCommands(
                new ParallelCommandGroup(new Command[] {
                        new PivotAngleCommand(pivotAngleSubsystem, PivotConstants.kPivotAmpPosition),
                        new AmpShootCommand(shooterSubsystem),
                        new AmpIndexerCommand(pivotAngleSubsystem, indexerSubsystem, shooterSubsystem,
                                ShooterConstants.kAmpRPM - 200,
                                PivotConstants.kPivotAmpPosition)
                }));
    }
}
