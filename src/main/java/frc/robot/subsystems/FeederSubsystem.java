// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.FeederConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkFlex m_feederShooterVortex;
    private final DigitalInput m_BeamBraker = new DigitalInput(FeederConstants.kBeamBreakerChannel);

    /** Create motor elements. */
    public FeederSubsystem() {

        ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
        feederTab.addBoolean("Beam Breaker Activated", () -> beamBreakerActivated());

        m_feederShooterVortex = new CANSparkFlex(FeederConstants.kFeederShooterCANId, MotorType.kBrushless);

        m_feederShooterVortex.restoreFactoryDefaults();

        m_feederShooterVortex.setInverted(true);
        m_feederShooterVortex.setIdleMode(IdleMode.kBrake);

        m_feederShooterVortex.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean beamBreakerActivated() {
        return m_BeamBraker.get();
    }

    /** Setting motor speeds. */
    public void startFeeder() {
        m_feederShooterVortex.set(FeederConstants.kFeederSpeed);
    }

    public void stopFeeder() {
        m_feederShooterVortex.set(0);
    }
}
