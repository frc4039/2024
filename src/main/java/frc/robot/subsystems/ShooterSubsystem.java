// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkFlex m_lowerShooterVortex;
  private final CANSparkFlex m_upperShooterVortex;
  private final CANSparkFlex m_feederShooterVortex;


 
 /**Create motor elements.*/
  public ShooterSubsystem() {
    m_lowerShooterVortex = new CANSparkFlex(ShooterConstants.lowerShooterCANId, MotorType.kBrushless);
    m_upperShooterVortex = new CANSparkFlex(ShooterConstants.upperShooterCANId, MotorType.kBrushless);
    m_feederShooterVortex = new CANSparkFlex(ShooterConstants.feederShooterCANId, MotorType.kBrushless);

    m_lowerShooterVortex.restoreFactoryDefaults();
    m_upperShooterVortex.restoreFactoryDefaults();
    m_feederShooterVortex.restoreFactoryDefaults();

    m_upperShooterVortex.setInverted(true);
    m_feederShooterVortex.setInverted(true);

    m_lowerShooterVortex.burnFlash();
    m_upperShooterVortex.burnFlash();
    m_feederShooterVortex.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**Setting motor speeds.*/
  public void shooterSpeedControl(double lowerSpeed, double upperSpeed, double speedLimit, double feederSpeed) {
    m_lowerShooterVortex.set(lowerSpeed * speedLimit);
    m_upperShooterVortex.set(upperSpeed * speedLimit);
    m_feederShooterVortex.set(feederSpeed);
  }

}
