// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex m_lowerShooterVortex;
    private final CANSparkFlex m_upperShooterVortex;
    private final SparkPIDController m_lowerShooterController;
    private final SparkPIDController m_upperShooterController;

    public ShooterSubsystem() {
        m_lowerShooterVortex = new CANSparkFlex(ShooterConstants.kLowerShooterCANId, MotorType.kBrushless);
        m_upperShooterVortex = new CANSparkFlex(ShooterConstants.kUpperShooterCANId, MotorType.kBrushless);

        m_lowerShooterVortex.restoreFactoryDefaults();
        m_upperShooterVortex.restoreFactoryDefaults();

        m_lowerShooterController = m_lowerShooterVortex.getPIDController();
        m_upperShooterController = m_upperShooterVortex.getPIDController();

        m_lowerShooterController.setP(ShooterConstants.kShooterP);
        m_lowerShooterController.setI(ShooterConstants.kShooterI);
        m_lowerShooterController.setD(ShooterConstants.kShooterD);
        m_lowerShooterController.setFF(ShooterConstants.kShooterFF);

        m_upperShooterController.setP(ShooterConstants.kShooterP);
        m_upperShooterController.setI(ShooterConstants.kShooterI);
        m_upperShooterController.setD(ShooterConstants.kShooterD);
        m_upperShooterController.setFF(ShooterConstants.kShooterFF);

        m_upperShooterVortex.setInverted(true);

        m_lowerShooterVortex.burnFlash();
        m_upperShooterVortex.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
}
