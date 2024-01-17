// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkFlex m_leftShooterVortex;
  private final CANSparkFlex m_rightShooterVortex;

 
 /**Create motor elements.*/
  public ShooterSubsystem() {
    m_leftShooterVortex = new CANSparkFlex(Constants.leftShooterCANId, MotorType.kBrushless);
    m_rightShooterVortex = new CANSparkFlex(Constants.rightShooterCANId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**Setting motor speeds.*/
  public void shooterSpeedControl(double leftSpeed, double rightSpeed, double speedLimit) {
    m_leftShooterVortex.set(leftSpeed * speedLimit);
    m_rightShooterVortex.set(rightSpeed * speedLimit);
  } 

}
