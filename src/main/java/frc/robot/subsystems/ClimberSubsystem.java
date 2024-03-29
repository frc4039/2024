// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.HardwareMonitor;

public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax m_climberLeaderMotor;
    private CANSparkMax m_climberFollowerMotor;
    private final SparkPIDController m_leftMotorController;
    private final SparkPIDController m_rightMotorController;
    private boolean debugging = true;

    // private Servo m_trapActuator;

    public ClimberSubsystem(HardwareMonitor hw) {
        m_climberLeaderMotor = CreateClimberMotor(ClimberConstants.kClimberLeaderMotorCANId);
        m_climberFollowerMotor = CreateClimberMotor(ClimberConstants.kClimberFollowerMotorCANId);
        m_climberFollowerMotor.burnFlash();
        m_climberLeaderMotor.burnFlash();

        m_leftMotorController = m_climberLeaderMotor.getPIDController();
        m_rightMotorController = m_climberFollowerMotor.getPIDController();

        m_leftMotorController.setP(ClimberConstants.kClimberP);
        m_leftMotorController.setI(ClimberConstants.kClimberI);
        m_leftMotorController.setD(ClimberConstants.kClimberD);
        m_leftMotorController.setFF(ClimberConstants.kClimberFF);

        m_rightMotorController.setP(ClimberConstants.kClimberP);
        m_rightMotorController.setI(ClimberConstants.kClimberI);
        m_rightMotorController.setD(ClimberConstants.kClimberD);
        m_rightMotorController.setFF(ClimberConstants.kClimberFF);

        hw.registerDevice(this, m_climberLeaderMotor);
        hw.registerDevice(this, m_climberFollowerMotor);

        if (debugging) {
            ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
            climberTab.addDouble("Leader Motor Current", () -> m_climberLeaderMotor.getOutputCurrent());
            climberTab.addDouble("Follower Motor Current", () -> m_climberFollowerMotor.getOutputCurrent());
        }
    }

    public void setClimbSpeed(double motorSpeed) {
        m_leftMotorController.setReference(motorSpeed, ControlType.kVelocity);
        m_rightMotorController.setReference(motorSpeed, ControlType.kVelocity);
    }

    public void setClimbPercentOutput(double percentOutput) {
        m_leftMotorController.setReference(percentOutput, ControlType.kVelocity);
        m_rightMotorController.setReference(percentOutput, ControlType.kVelocity);
    }

    /*
     * public double getLeftTriggerOutput() {
     * return
     * }
     * 
     * public double getRightTriggerOutput() {
     * return
     * }
     */

    public void stop() {
        m_climberLeaderMotor.set(0);
        m_climberFollowerMotor.set(0);
    }

    private CANSparkMax CreateClimberMotor(int motorCANId) {
        CANSparkMax motor = new CANSparkMax(motorCANId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(ClimberConstants.kClimberSmartCurrentLimit);
        return motor;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
