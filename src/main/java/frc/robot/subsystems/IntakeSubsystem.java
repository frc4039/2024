package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkFlex m_intakeUpperMotor;
    private CANSparkMax m_intakeLowerMotor;

    public IntakeSubsystem() {
        // Upper Motor
        m_intakeUpperMotor = new CANSparkFlex(IntakeConstants.kIntakeUpperMotorCANID, MotorType.kBrushless);
        m_intakeUpperMotor.restoreFactoryDefaults();
        m_intakeUpperMotor.setInverted(false);
        m_intakeUpperMotor.setSmartCurrentLimit(40, 40);
        m_intakeUpperMotor.burnFlash();

        // Lower Motor
        m_intakeLowerMotor = new CANSparkMax(IntakeConstants.kIntakeLowerMotorCANID, MotorType.kBrushless);
        m_intakeLowerMotor.restoreFactoryDefaults();
        m_intakeLowerMotor.setSmartCurrentLimit(100);
        m_intakeLowerMotor.setInverted(false);
        m_intakeLowerMotor.burnFlash();

        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        intakeTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);
    }

    public void spinIntakeMotor(double spinSpeedUpperMotor, double spinSpeedLowerMotor) {
        m_intakeUpperMotor.set(spinSpeedUpperMotor);
        m_intakeLowerMotor.set(spinSpeedLowerMotor);
    }

    @Override
    public void periodic() {
    }
}
