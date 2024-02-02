package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkFlex m_intakeUpperMotor;
    private CANSparkFlex m_intakeLowerMotor;

    public IntakeSubsystem() {
        m_intakeUpperMotor = createIntakeMotor(IntakeConstants.kIntakeUpperMotorCANID, false);
        m_intakeLowerMotor = createIntakeMotor(IntakeConstants.kIntakeLowerMotorCANID, true);
    }

    public void spinIntakeMotor(double spinSpeed) {
        m_intakeUpperMotor.set(spinSpeed);
        m_intakeLowerMotor.set(spinSpeed);
    }

    private CANSparkFlex createIntakeMotor(int motorId, boolean isInverted) {
        CANSparkFlex intakeMotor = new CANSparkFlex(motorId, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(isInverted);
        intakeMotor.burnFlash();

        return intakeMotor;
    }

    @Override
    public void periodic() {
    }
}
