// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotConstants;

public class PivotAngleSubsystem extends SubsystemBase {
    private final CANSparkMax m_pivotSparkMax;
    // private final CANSparkMax m_pivot2SparkMax;
    private final SparkPIDController m_pivotPIDController;
    private final RelativeEncoder m_pivotEncoder;

    /** Creates a new PivotAngle. */
    public PivotAngleSubsystem() {
        m_pivotSparkMax = new CANSparkMax(PivotConstants.kPivotCANId, MotorType.kBrushless);
        // m_pivot2SparkMax = new CANSparkMax(PivotConstants.kPivot2CANId,
        // MotorType.kBrushed);

        m_pivotSparkMax.restoreFactoryDefaults();
        // m_pivot2SparkMax.restoreFactoryDefaults();
        // _pivot2SparkMax.follow(m_pivotSparkMax, true);

        m_pivotEncoder = m_pivotSparkMax.getEncoder();
        m_pivotPIDController = m_pivotSparkMax.getPIDController();
        m_pivotPIDController.setFeedbackDevice(m_pivotEncoder);

        m_pivotEncoder.setPositionConversionFactor(PivotConstants.kPivotEncoderPositionFactor);
        m_pivotEncoder.setVelocityConversionFactor(PivotConstants.kPivotEncoderVelocityFactor);

        // m_pivotEncoder.setInverted(PivotConstants.kPivotEncoderInverted);

        m_pivotPIDController.setP(PivotConstants.kPivotP);
        m_pivotPIDController.setI(PivotConstants.kPivotI);
        m_pivotPIDController.setD(PivotConstants.kPivotD);
        m_pivotPIDController.setFF(PivotConstants.kPivotFF);
        m_pivotPIDController.setOutputRange(PivotConstants.kPivotMinOutput,
                PivotConstants.kPivotMaxOutput);

        m_pivotSparkMax.burnFlash();
        // m_pivot2SparkMax.burnFlash();

        ShuffleboardTab pivotAngleTab = Shuffleboard.getTab("PivotAngle");
        pivotAngleTab.addDouble("encoder", () -> m_pivotEncoder.getPosition() * 360.0);

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
