// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.HardwareMonitor;

public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax m_climberLeaderMotor;
    private CANSparkMax m_climberFollowerMotor;
    private Servo m_trapActuator;

    public ClimberSubsystem(HardwareMonitor hw) {
        m_climberLeaderMotor = CreateClimberMotor(ClimberConstants.kClimberLeaderMotorCANId);
        m_climberFollowerMotor = CreateClimberMotor(ClimberConstants.kClimberFollowerMotorCANId);
        m_climberFollowerMotor.follow(m_climberLeaderMotor, true);
        m_trapActuator = new Servo(1);

        hw.registerDevice(this, m_climberLeaderMotor);
        hw.registerDevice(this, m_climberFollowerMotor);
    }

    public void deployFlapTrap() {
        m_trapActuator.set(0.0);
    }

    public void unDeployFlapTrap() {
        m_trapActuator.set(1.0);
    }

    public void setClimbSpeed(double motorSpeed) {
        m_climberLeaderMotor.set(motorSpeed);
    }

    public void stop() {
        m_climberLeaderMotor.set(0);
        m_climberLeaderMotor.setIdleMode(IdleMode.kBrake);
        m_climberFollowerMotor.setIdleMode(IdleMode.kBrake);
    }

    private CANSparkMax CreateClimberMotor(int motorCANId) {
        CANSparkMax motor = new CANSparkMax(motorCANId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(ClimberConstants.kClimberSmartCurrentLimit);
        motor.burnFlash();
        return motor;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
