// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.HardwareMonitor;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex m_lowerShooterVortex;
    private final CANSparkFlex m_upperShooterVortex;
    private final SparkPIDController m_lowerShooterController;
    private final SparkPIDController m_upperShooterController;
    private final RelativeEncoder m_lowerShooterEncoder;
    private final RelativeEncoder m_upperShooterEncoder;

    public ShooterSubsystem(HardwareMonitor hw) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

        m_lowerShooterVortex = new CANSparkFlex(ShooterConstants.kLowerShooterCANId, MotorType.kBrushless);
        m_upperShooterVortex = new CANSparkFlex(ShooterConstants.kUpperShooterCANId, MotorType.kBrushless);

        m_lowerShooterVortex.restoreFactoryDefaults();
        m_upperShooterVortex.restoreFactoryDefaults();

        m_lowerShooterController = m_lowerShooterVortex.getPIDController();
        m_upperShooterController = m_upperShooterVortex.getPIDController();

        m_lowerShooterEncoder = m_lowerShooterVortex.getEncoder();
        m_upperShooterEncoder = m_upperShooterVortex.getEncoder();

        m_lowerShooterController.setP(ShooterConstants.kShooterP);
        m_lowerShooterController.setI(ShooterConstants.kShooterI);
        m_lowerShooterController.setD(ShooterConstants.kShooterD);
        m_lowerShooterController.setFF(ShooterConstants.kShooterFF);

        m_upperShooterController.setP(ShooterConstants.kShooterP);
        m_upperShooterController.setI(ShooterConstants.kShooterI);
        m_upperShooterController.setD(ShooterConstants.kShooterD);
        m_upperShooterController.setFF(ShooterConstants.kShooterFF);

        m_upperShooterVortex.setInverted(false);
        m_lowerShooterVortex.setInverted(true);

        m_lowerShooterVortex.burnFlash();
        m_upperShooterVortex.burnFlash();

        hw.registerDevice(this, m_lowerShooterVortex);
        hw.registerDevice(this, m_upperShooterVortex);

        shooterTab.addDouble("Upper Speed (RPM)", () -> m_upperShooterEncoder.getVelocity());
        shooterTab.addDouble("Lower Speed (RPM)", () -> m_lowerShooterEncoder.getVelocity());
        shooterTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);
        SmartDashboard.putNumber("Speed Setpoint", ShooterConstants.kShooterRPM);
    }

    @Override
    public void periodic() {

        double manSpeed = SmartDashboard.getNumber("Speed Setpoint", 500);
        if ((manSpeed != ShooterConstants.kShooterRPM)) {
            ShooterConstants.kShooterRPM = manSpeed;
        }

    }

    /** Setting motor speeds. */
    public void shooterSpeedControl(double lowerSpeed, double upperSpeed, double speedLimit) {
        m_lowerShooterVortex.set(lowerSpeed * speedLimit);
        m_upperShooterVortex.set(upperSpeed * speedLimit);
    }

    /**
     * Set the shooter to run at a particular RPM.
     * 
     * @param rpm The speed to run the shooter at in rotations per minute.
     */
    public void shooterPID(double rpm) {
        m_lowerShooterController.setReference(rpm, ControlType.kVelocity);
        m_upperShooterController.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Set the shooter to run at a set RPM in amp mode.
     * 
     * @param rpm The speed to run the shooter at in rotations per minute.
     */
    public void ampPID(double rpm) {
        m_lowerShooterController.setReference(rpm, ControlType.kVelocity);
        m_upperShooterController.setReference(-rpm, ControlType.kVelocity);
    }

    /**
     * Gets the speed of the upper shooter motor (in RPM).
     * 
     * @return The velocity of the upper shooter motor (in RPM).
     */
    public double getShooterSpeed() {
        return m_upperShooterEncoder.getVelocity();
    }
}
