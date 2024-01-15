// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.co;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.pheonix.sensors.*;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX m_leftShooterFalcon;
  private final TalonFX m_rightShooterFalcon;

  private final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);


  public ShooterSubsystem(int leftShooterCANId, int rightShooterCANId) {
    m_leftShooterFalcon = new TalonFX(leftShooterCANId);
    m_rightShooterFalcon = new TalonFX(rightShooterCANId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooterSpeedControl(double leftSpeed, double rightSpeed, double speedLimit) {
    m_leftShooterFalcon.setControl(m_leftRequest.withOutput(leftSpeed * speedLimit));
    m_rightShooterFalcon.setControl(m_rightRequest.withOutput(rightSpeed * speedLimit));
  } 
}
