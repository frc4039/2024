// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.utils.Sensors;

/** Add your docs here. */
public class BlinkinSubsystem extends SubsystemBase {
    private Spark m_BlinkinStrip;

    private boolean HasNoteCurrent;
    private boolean HasNotePrevious;

    private double StartTime;
    private double newColour;
    private double prevColour;

    public BlinkinSubsystem() {

        m_BlinkinStrip = new Spark(BlinkinConstants.kBlinkinPWMPort);
        HasNotePrevious = false;
        prevColour = BlinkinConstants.kColourValueRainbow;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        HasNoteCurrent = Sensors.BeamBreakerIsBroken();
        if (HasNoteCurrent && !HasNotePrevious) { // Just recieved the note so start blinking with timer
            newColour = BlinkinConstants.kColourValueGreenFlashing;
            StartTime = Timer.getFPGATimestamp();
        } else if (HasNoteCurrent && HasNotePrevious) { // Check if timmer has elapsed and change to solid green
            if ((Timer.getFPGATimestamp() - StartTime) > BlinkinConstants.BlinkTime) {
                newColour = BlinkinConstants.kColourValueGreen;
            }
        } else { // No note so change to rainbow
            newColour = BlinkinConstants.kColourValueRainbow;
        }
        HasNotePrevious = HasNoteCurrent;

        if (prevColour != newColour) {
            SetColour(newColour);
            prevColour = newColour;
        }
    }

    public void SetColour(double colourValue) {
        if ((colourValue >= -1.0) && (colourValue <= 1.0)) {
            m_BlinkinStrip.set(colourValue);
        }
    }

}