// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FeederSubsystem extends SubsystemBase {
  private final CANSparkFlex m_feederShooterVortex;


 
 /**Create motor elements.*/
  public FeederSubsystem() {
    m_feederShooterVortex = new CANSparkFlex(ShooterConstants.feederShooterCANId, MotorType.kBrushless);

    m_feederShooterVortex.restoreFactoryDefaults();

    m_feederShooterVortex.setInverted(true);

    m_feederShooterVortex.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**Setting motor speeds.*/
  public void feederSpeedControl(double feederSpeed) {
    m_feederShooterVortex.set(feederSpeed);
  }
}
