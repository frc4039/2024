// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HumanPlayerIntakeCommand extends Command {
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;

    /** Creates a new HumanPlayerIntakeCommand. */
    public HumanPlayerIntakeCommand(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(shooter);
        addRequirements(feeder);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shooterSpeedControl(ShooterConstants.kHumanPlayerLowerMotorSpeed,
                ShooterConstants.kHumanPlayerUpperMotorSpeed,
                ShooterConstants.kShooterHumanPlayerSpeedLimit);
                feeder.startFeeder(FeederConstants.kFeederHumanPlayerSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shooterSpeedControl(0, 0, 0);
        feeder.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
