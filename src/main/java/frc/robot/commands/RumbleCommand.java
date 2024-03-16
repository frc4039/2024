// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Sensors;

public class RumbleCommand extends Command {
    private Joystick driverControler;
    private Joystick operatorControler;

    double StartTime;
    boolean TimerStarted;

    /** Creates a new rumble. */
    public RumbleCommand(Joystick driverControler, Joystick operatorControler) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driverControler = driverControler;
        this.operatorControler = operatorControler;
        // m_Indexer = Indexer;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // controler ruble intialize
        TimerStarted = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Sensors.BeamBreakerIsBroken() && !TimerStarted) {
            driverControler.setRumble(RumbleType.kBothRumble, 1.0);
            operatorControler.setRumble(RumbleType.kBothRumble, 1.0);
            TimerStarted = true;
            StartTime = Timer.getFPGATimestamp();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driverControler.setRumble(RumbleType.kBothRumble, 0.0);
        operatorControler.setRumble(RumbleType.kBothRumble, 0.0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (TimerStarted && ((Timer.getFPGATimestamp() - StartTime) > 0.5)) ? true : false;
    }
}
