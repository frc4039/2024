// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotAngleSubsystem extends SubsystemBase {
    private final CANSparkMax m_pivotSparkMax;
    private final SparkPIDController m_pivotPIDController;
    private final AbsoluteEncoder m_pivotEncoder;

    /** Creates a new PivotAngle. */
    public PivotAngleSubsystem() {
        m_pivotSparkMax = new CANSparkMax(PivotConstants.kPivotCANId, MotorType.kBrushless);

        m_pivotSparkMax.restoreFactoryDefaults();
        m_pivotSparkMax.setSmartCurrentLimit(50);

        m_pivotEncoder = m_pivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_pivotPIDController = m_pivotSparkMax.getPIDController();
        m_pivotPIDController.setFeedbackDevice(m_pivotEncoder);

        m_pivotEncoder.setPositionConversionFactor(PivotConstants.kPivotEncoderPositionFactor);
        m_pivotEncoder.setVelocityConversionFactor(PivotConstants.kPivotEncoderVelocityFactor);

        m_pivotEncoder.setInverted(PivotConstants.kPivotEncoderInverted);
        m_pivotEncoder.setZeroOffset(PivotConstants.kPivotOffset * 360.0);

        m_pivotPIDController.setP(PivotConstants.kPivotP);
        m_pivotPIDController.setI(PivotConstants.kPivotI);
        m_pivotPIDController.setD(PivotConstants.kPivotD);
        m_pivotPIDController.setFF(PivotConstants.kPivotFF);
        m_pivotPIDController.setOutputRange(PivotConstants.kPivotMinOutput,
                PivotConstants.kPivotMaxOutput);

        m_pivotSparkMax.burnFlash();

        ShuffleboardTab pivotAngleTab = Shuffleboard.getTab("PivotAngle");
        pivotAngleTab.addDouble("encoder", () -> m_pivotEncoder.getPosition() * 360.0);
        pivotAngleTab.addDouble("built-in encoder", () -> m_pivotSparkMax.getEncoder().getPosition() * 360.0);
        pivotAngleTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);
    }

    public void setDesiredAngle(double desiredAngle) {
        m_pivotPIDController.setReference(desiredAngle / 360.0,
                CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
