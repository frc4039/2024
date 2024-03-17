// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.utils.Sensors;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class BlinkinSubsystem extends SubsystemBase {
    private Spark m_BlinkinStrip;

    private boolean HasNoteCurrent;
    private boolean HasNotePrevious;

    private Timer BlinkTimer;

    public BlinkinSubsystem() {

        m_BlinkinStrip = new Spark(BlinkinConstants.kBlinkinPWMPort);
        HasNotePrevious = false;
    }

    // Colours for states
    // High Auto Green
    // High podium flashing green
    // High sub Flasing yellow
    // Low orange
    // CLIMB rainbow
    // flash white when pickingup note

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        HasNoteCurrent = Sensors.BeamBreakerIsBroken();

        if (HasNoteCurrent == true && HasNotePrevious == false) {
            BlinkTimer.reset();
            BlinkTimer.start();
        }
        if (HasNoteCurrent == true && BlinkTimer.get() < BlinkinConstants.BlinkPeriod) {
            // blink white
        } else {
            switch (RobotContainer.scoringState) {
                case HIGH:
                    // Colour Green
                    break;
                case LOW:
                    // Colour Orange
                    break;
                case PodiumShoot:
                    // colour flashing green
                    break;
                case SubwooferShoot:
                    // colour flashing yellow
                    break;
                case CLIMB:
                    // colour rainbow
                    break;
                default:
                    // colour rainbow
                    break;
            }
        }
    }

    public void SetColour(double colourValue) {
        if ((colourValue >= -1.0) && (colourValue <= 1.0)) {
            m_BlinkinStrip.set(colourValue);
        }
    }

}