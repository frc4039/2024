package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.HardwareMonitor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ActivateTrapSubsystem extends SubsystemBase {

    private Servo TrapActuatorRight;
    private Servo TrapActuatorLeft;

    public ActivateTrapSubsystem(HardwareMonitor hw) {
        TrapActuatorRight = new Servo(ClimberConstants.TrapActuatorRightPort);
        TrapActuatorLeft = new Servo(ClimberConstants.TrapActuatorLeftPort);
        TrapActuatorRight.setBoundsMicroseconds(2000, 1990, 1500, 1010, 1000);
        TrapActuatorLeft.setBoundsMicroseconds(2000, 1990, 1500, 1010, 1000);
        TrapActuatorRight.setPosition(0.0);
        TrapActuatorLeft.setPosition(1.0);
    }

    public void Release() {
        TrapActuatorRight.setPosition(1.0);
        TrapActuatorLeft.setPosition(0.0);
    }

    public void Reset() {
        TrapActuatorRight.setPosition(0.0);
        TrapActuatorLeft.setPosition(1.0);
    }
}
