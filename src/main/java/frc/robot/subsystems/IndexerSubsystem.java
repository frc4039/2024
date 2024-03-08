// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.utils.HardwareMonitor;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax m_indexerNeo550;
    private DigitalInput m_beamBreak = IndexerConstants.kBeamBreakDIO;

    /** Create motor elements. */
    public IndexerSubsystem(HardwareMonitor hw) {
        m_indexerNeo550 = new CANSparkMax(IndexerConstants.kIndexerCANID, MotorType.kBrushless);

        m_indexerNeo550.restoreFactoryDefaults();

        m_indexerNeo550.setInverted(false);
        m_indexerNeo550.setIdleMode(IdleMode.kBrake);

        m_indexerNeo550.burnFlash();

        hw.registerDevice(this, m_indexerNeo550);

        ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
        indexerTab.addBoolean("Beam Breaker Activated", () -> hasNote());
        indexerTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean hasNote() {
        return !m_beamBreak.get();
    }

    /** Setting motor speeds. */
    public void start(double percentOutput) {
        m_indexerNeo550.set(percentOutput);
    }

    public void stop() {
        m_indexerNeo550.set(0);
    }
}
