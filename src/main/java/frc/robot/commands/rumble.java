// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IndexerSubsystem;

public class rumble extends Command {
    private Joystick driverControler;
    private Joystick operatorControler;
    double m_starttime;
    boolean TimerStarted = false;
    IndexerSubsystem m_indexer;

    /** Creates a new rumble. */
    public rumble(Joystick driverControler, Joystick operatorControler, IndexerSubsystem indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driverControler = driverControler;
        this.operatorControler = operatorControler;
        m_indexer = indexer;
        addRequirements(m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // controler ruble intialize
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_indexer.hasNote() && !TimerStarted) {
            TimerStarted = true;
            m_starttime = Timer.getFPGATimestamp();
            driverControler.setRumble(RumbleType.kBothRumble, 1.0);
            operatorControler.setRumble(RumbleType.kBothRumble, 1.0);
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
        if (TimerStarted) {
            return (((Timer.getFPGATimestamp() - m_starttime) > 0.5) ? true : false);
        } else {
            return false;
        }
    }
}
