package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX m_intakeSpinningMotor;

    public IntakeSubsystem() {
        m_intakeSpinningMotor = new TalonFX(IntakeConstants.kIntakeMotorCANID);

        TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();
        m_intakeSpinningMotor.getConfigurator().apply(m_intakeConfig);
    }

    public void spinIntakeMotor(double spinSpeed) {
        m_intakeSpinningMotor.set(spinSpeed);
    }

    @Override
    public void periodic() {
    }
}
