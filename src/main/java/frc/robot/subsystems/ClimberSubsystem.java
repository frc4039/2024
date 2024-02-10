// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax m_climberLeaderMotor;
    private CANSparkMax m_climberFollowerMotor;

    public ClimberSubsystem() {
        m_climberLeaderMotor = CreateClimberMotor(ClimberConstants.kClimberLeaderMotorCANId, false);
        m_climberFollowerMotor = CreateClimberMotor(ClimberConstants.kClimberFollowerMotorCANId, true);
        m_climberFollowerMotor.follow(m_climberLeaderMotor);
    }

    public void ClimbOnStage(double motorSpeed) {
        m_climberLeaderMotor.set(motorSpeed);
    }

    public void StopClimbing() {
        m_climberLeaderMotor.set(0);
    }

    private CANSparkMax CreateClimberMotor(int motorCANId, boolean isInverted) {
        CANSparkMax motor = new CANSparkMax(motorCANId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(ClimberConstants.kClimberSmartCurrentLimit);
        motor.setInverted(isInverted);
        motor.burnFlash();
        return motor;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
