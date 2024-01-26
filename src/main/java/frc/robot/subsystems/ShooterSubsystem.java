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
  private final CANSparkFlex m_leftShooterVortex;
  private final CANSparkFlex m_rightShooterVortex;
  private final SparkPIDController m_leftShooterController;
  private final SparkPIDController m_rightShooterController;

 
 /**Create motor elements.*/
  public ShooterSubsystem() {
    m_leftShooterVortex = new CANSparkFlex(ShooterConstants.leftShooterCANId, MotorType.kBrushless);
    m_rightShooterVortex = new CANSparkFlex(ShooterConstants.rightShooterCANId, MotorType.kBrushless);

    m_leftShooterVortex.restoreFactoryDefaults();
    m_rightShooterVortex.restoreFactoryDefaults();

    m_leftShooterController = m_leftShooterVortex.getPIDController();
    m_rightShooterController = m_rightShooterVortex.getPIDController();

    m_leftShooterController.setP(ShooterConstants.kShooterP);
    m_leftShooterController.setI(ShooterConstants.kShooterI);
    m_leftShooterController.setD(ShooterConstants.kShooterD);
    m_leftShooterController.setFF(ShooterConstants.kShooterFF);

    m_rightShooterController.setP(ShooterConstants.kShooterP);
    m_rightShooterController.setI(ShooterConstants.kShooterI);
    m_rightShooterController.setD(ShooterConstants.kShooterD);
    m_rightShooterController.setFF(ShooterConstants.kShooterFF);

    m_rightShooterVortex.setInverted(true);

    m_leftShooterVortex.burnFlash();
    m_rightShooterVortex.burnFlash();
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

  public void shooterPID(double rpm) {
    m_leftShooterController.setReference(rpm, ControlType.kVelocity);
    m_rightShooterController.setReference(rpm, ControlType.kVelocity);
  }

}
