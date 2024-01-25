package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkFlex m_intakeSpinningMotor;


    public IntakeSubsystem() {
        m_intakeSpinningMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorCANID,MotorType.kBrushless);

        m_intakeSpinningMotor.restoreFactoryDefaults();
        m_intakeSpinningMotor.burnFlash();
    }

    public void spinIntakeMotor(double spinSpeed){
        m_intakeSpinningMotor.set(spinSpeed);
    }

    @Override
    public void periodic() {}
}
