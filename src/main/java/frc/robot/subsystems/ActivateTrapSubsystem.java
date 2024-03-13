package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.HardwareMonitor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ActivateTrapSubsystem extends SubsystemBase {

    private Servo TrapActuator;

    public ActivateTrapSubsystem(HardwareMonitor hw) {
        TrapActuator = new Servo(ClimberConstants.TrapActuatorPort);
        TrapActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        TrapActuator.setPosition(0);
    }

    public void Release() {
        TrapActuator.setPosition(1.0);
    }

    public void Reset() {
        TrapActuator.setPosition(0);
    }
}
