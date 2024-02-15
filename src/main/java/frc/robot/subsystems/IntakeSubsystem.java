package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeLowerMotor;

    public IntakeSubsystem() {

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

    public void spinIntakeMotor(double spinSpeedLowerMotor) {
        m_intakeLowerMotor.set(spinSpeedLowerMotor);
    }

    @Override
    public void periodic() {
    }
}
