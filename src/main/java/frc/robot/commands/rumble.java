// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Rumble extends Command {
    private Joystick driverControler;
    private Joystick operatorControler;

    /** Creates a new rumble. */
    public Rumble(Joystick driverControler, Joystick operatorControler) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driverControler = driverControler;
        this.operatorControler = operatorControler;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // controler ruble intialize
        driverControler.setRumble(RumbleType.kBothRumble, 1.0);
        operatorControler.setRumble(RumbleType.kBothRumble, 1.0);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
        return false;
    }
}
