// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerShootParallelCommandGroup extends ParallelCommandGroup {
    /** Creates a new SpeakerShootParallelCommandGroup. */
    public SpeakerShootParallelCommandGroup(DriveSubsystem driveSubsystem,
            ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
            PivotAngleSubsystem pivotAngleSubsystem,
            DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {

        addCommands(
                new AimAtSpeakerCommand(driveSubsystem, xSpeedSupplier, ySpeedSupplier),
                new ShootCommand(shooterSubsystem),
                new PivotToShootCommand(pivotAngleSubsystem, driveSubsystem));

        // check process vs tolerances
        // flash leds
    }
}
