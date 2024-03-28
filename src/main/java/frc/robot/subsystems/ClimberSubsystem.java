// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.HardwareMonitor;

public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax m_climberLeaderMotor;
    private CANSparkMax m_climberFollowerMotor;
    private boolean debugging = true;

    // private Servo m_trapActuator;

    public ClimberSubsystem(HardwareMonitor hw) {
        m_climberLeaderMotor = CreateClimberMotor(ClimberConstants.kClimberLeaderMotorCANId, true);
        m_climberFollowerMotor = CreateClimberMotor(ClimberConstants.kClimberFollowerMotorCANId, false);
        // m_climberFollowerMotor.follow(m_climberLeaderMotor, true);
        m_climberFollowerMotor.burnFlash();
        m_climberLeaderMotor.burnFlash();

        hw.registerDevice(this, m_climberLeaderMotor);
        hw.registerDevice(this, m_climberFollowerMotor);

        if (debugging) {
            ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
            climberTab.addDouble("Leader Motor Current", () -> m_climberLeaderMotor.getOutputCurrent());
            climberTab.addDouble("Follower Motor Current", () -> m_climberFollowerMotor.getOutputCurrent());
        }
    }

    public void setClimbSpeed(double motorSpeed, double bias) {
        m_climberLeaderMotor.set(motorSpeed + bias * ClimberConstants.kClimberBiasLimit);
        m_climberFollowerMotor.set(motorSpeed - bias * ClimberConstants.kClimberBiasLimit);
    }

    public void stop() {
        m_climberLeaderMotor.set(0);
        m_climberFollowerMotor.set(0);
    }

    private CANSparkMax CreateClimberMotor(int motorCANId, boolean inverted) {
        CANSparkMax motor = new CANSparkMax(motorCANId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(inverted);
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
