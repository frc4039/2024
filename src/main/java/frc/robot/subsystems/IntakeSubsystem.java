package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.HardwareMonitor;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotor;

    public IntakeSubsystem(HardwareMonitor hw) {
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setSmartCurrentLimit(100);
        m_intakeMotor.setInverted(false);
        m_intakeMotor.burnFlash();

        hw.registerDevice(this, m_intakeMotor);

        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        intakeTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);
        intakeTab.addDouble("Current", () -> m_intakeMotor.getOutputCurrent())
                .withPosition(7, 1)
                .withSize(1, 1);
    }

    public void spinIntakeMotor(double spinSpeedMotor) {
        m_intakeMotor.set(spinSpeedMotor);
    }

    @Override
    public void periodic() {
    }

    public double getOutputCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }
}
