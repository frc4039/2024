// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.ScoringState;
import frc.robot.utils.Sensors;

/** Add your docs here. */
public class BlinkinSubsystem extends SubsystemBase {
    private Spark m_BlinkinStrip;

    private boolean HasNoteCurrent;
    private boolean HasNotePrevious;

    private Timer BlinkTimer;
    // private Double CurrColour = -.99;
    private Supplier<ScoringState> State;
    // private ScoringState PrevScoringState = ScoringState.HIGH;

    public BlinkinSubsystem(Supplier<ScoringState> State) {

        m_BlinkinStrip = new Spark(BlinkinConstants.kBlinkinPWMPort);
        m_BlinkinStrip.setSafetyEnabled(HasNoteCurrent);
        HasNotePrevious = false;
        this.State = State;
        BlinkTimer = new Timer();
        BlinkTimer.stop();
        BlinkTimer.reset();
        // ShuffleboardTab driveTab = Shuffleboard.getTab("LED Strip");

        // driveTab.addDouble("Colour", () -> GetCurrColour())
        // .withPosition(0, 0);

    }

    // Colours for states
    // High Auto Green .17 colour 2 .76
    // High podium flashing green .35 colour 2
    // High sub Flasing yellow -.07 strobe gold solid .66 .68
    // Low orange .63
    // CLIMB rainbow
    // flash white when pickingup note .15 colour 1 -.05 -.11 flashing red

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        HasNoteCurrent = Sensors.BeamBreakerIsBroken();

        if (HasNoteCurrent == true && HasNotePrevious == false) {
            BlinkTimer.reset();
            BlinkTimer.start();
        }
        if (HasNoteCurrent == true && BlinkTimer.get() < BlinkinConstants.BlinkTime) {
            // blink white
            SetColour(BlinkinConstants.kColourWhiteFlash);
        } else {
            switch (State.get()) {
                case HIGH: // Colour Green
                    SetColour(BlinkinConstants.kColourGreen);
                    break;
                case LOW: // Colour Orange
                    SetColour(BlinkinConstants.kColourOrange);
                    break;
                case PodiumShoot: // Hot pink
                    SetColour(BlinkinConstants.kColourHotPink);
                    break;
                case SubwooferShoot: // colour Aqua
                    SetColour(BlinkinConstants.kColourAqua);
                    break;
                case CLIMB: // Colour Rainbow
                default:
                    SetColour(BlinkinConstants.kColourValueRainbow);
                    break;

            }
        }
        HasNotePrevious = HasNoteCurrent;
    }

    public void SetColour(double colourValue) {
        if ((colourValue >= -1.0) && (colourValue <= 1.0)) {
            m_BlinkinStrip.set(colourValue);

        }
    }

}