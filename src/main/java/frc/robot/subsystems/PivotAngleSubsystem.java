// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.PivotConstants;
import frc.robot.utils.HardwareMonitor;

public class PivotAngleSubsystem extends SubsystemBase {
    private final CANSparkMax m_pivotSparkMax;
    private final CANSparkMax m_pivotFollowerSparkMax;
    private final SparkPIDController m_pivotPIDController;
    private final AbsoluteEncoder m_pivotEncoder;

    // Motion Profile. Units are degrees per second and degrees per second per
    // second.
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(200.0,
            900.0);
    private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);

    // Feedforward is used to match the profile's desired velocity.
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.204, 0.0447, 0.00279);

    // Goal and current setpoint for following the motion profile.
    private TrapezoidProfile.State m_setpoint = null;
    private TrapezoidProfile.State m_goal = null;

    /** Creates a new PivotAngle. */
    public PivotAngleSubsystem(HardwareMonitor hw) {
        m_pivotSparkMax = new CANSparkMax(PivotConstants.kPivotCANId, MotorType.kBrushless);
        m_pivotFollowerSparkMax = new CANSparkMax(PivotConstants.kPivotFollowerCANId, MotorType.kBrushless);

        m_pivotSparkMax.restoreFactoryDefaults();
        m_pivotFollowerSparkMax.restoreFactoryDefaults();
        m_pivotSparkMax.setSmartCurrentLimit(50);
        m_pivotFollowerSparkMax.setSmartCurrentLimit(50);

        m_pivotSparkMax.setInverted(false);

        m_pivotEncoder = m_pivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        m_pivotPIDController = m_pivotSparkMax.getPIDController();
        m_pivotPIDController.setFeedbackDevice(m_pivotEncoder);

        m_pivotEncoder.setPositionConversionFactor(PivotConstants.kPivotEncoderPositionFactor);
        m_pivotEncoder.setVelocityConversionFactor(PivotConstants.kPivotEncoderVelocityFactor);

        m_pivotEncoder.setInverted(PivotConstants.kPivotEncoderInverted);
        m_pivotEncoder.setZeroOffset(PivotConstants.kPivotOffset);

        m_pivotPIDController.setP(PivotConstants.kPivotP);
        m_pivotPIDController.setI(PivotConstants.kPivotI);
        m_pivotPIDController.setD(PivotConstants.kPivotD);
        m_pivotPIDController.setFF(PivotConstants.kPivotFF);
        m_pivotPIDController.setOutputRange(PivotConstants.kPivotMinOutput,
                PivotConstants.kPivotMaxOutput);

        m_pivotFollowerSparkMax.follow(m_pivotSparkMax, true);

        m_pivotSparkMax.burnFlash();
        m_pivotFollowerSparkMax.burnFlash();

        hw.registerDevice(this, m_pivotSparkMax);
        hw.registerDevice(this, m_pivotFollowerSparkMax);

        ShuffleboardTab pivotAngleTab = Shuffleboard.getTab("PivotAngle");
        pivotAngleTab.addDouble("encoder", this::getPitch);
        pivotAngleTab.addDouble("velocity", this::getPitchAngularVelocity);
        pivotAngleTab.addDouble("voltage", () -> m_pivotSparkMax.getAppliedOutput() * m_pivotSparkMax.getBusVoltage());
        pivotAngleTab.addDouble("built-in encoder", () -> m_pivotSparkMax.getEncoder().getPosition());
        pivotAngleTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);

        SmartDashboard.putNumber("Angle Setpoint", PivotConstants.kPivotSubwooferPosition);

    }

    /** Rotate to the desired angle using a motion profile. */
    public void setDesiredAngle(double goalAngle) {
        m_goal = new TrapezoidProfile.State(goalAngle, 0);
    }

    public double getPitch() {
        return m_pivotEncoder.getPosition();
    }

    public double getPitchAngularVelocity() {
        return m_pivotEncoder.getVelocity();
    }

    /** Stop outputting to the motor. */
    public void stop() {
        m_goal = null;
    }

    @Override
    public void periodic() {
        double manAngle = SmartDashboard.getNumber("Angle Setpoint", 500);
        if ((manAngle != PivotConstants.kPivotSubwooferPosition)) {
            PivotConstants.kPivotSubwooferPosition = manAngle;
        }
        // For safety, stop the pivot whenever the robot is disabled.
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (m_goal != null) {
            // Reset setpoint if the motor has been stopped.
            if (m_setpoint == null) {
                m_setpoint = new TrapezoidProfile.State(getPitch(), getPitchAngularVelocity());
            }

            // Calculate the next state in the motion profile.
            m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);

            double feedforward = m_feedforward.calculate(m_setpoint.velocity);
            m_pivotPIDController.setReference(m_setpoint.position, ControlType.kPosition, 0, feedforward);
        } else {
            m_pivotPIDController.setReference(0, ControlType.kDutyCycle);
            m_setpoint = null;
        }
    }

    /**
     * Returns the SysIdRoutine factory for building commands required for
     * system identification.
     */
    public SysIdRoutine getSysId() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(0.5), Units.Volts.of(3.0), null),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            m_pivotSparkMax.setVoltage(voltage.in(Units.Volts));
                        },
                        (log) -> {
                            log.motor("pivotLeader")
                                    .voltage(Units.Volts
                                            .of(m_pivotSparkMax.getAppliedOutput() * m_pivotSparkMax.getBusVoltage()))
                                    .angularPosition(Units.Rotations.of(getPitch()))
                                    .angularVelocity(Units.RotationsPerSecond.of(getPitchAngularVelocity()));
                        }, this));
    }
}
