package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Logger for any hardware errors. */
public class HardwareMonitor implements AutoCloseable {
    private HashMap<Subsystem, List<Device>> devices = new HashMap<Subsystem, List<Device>>();
    private static Subsystem defaultSubsystem = new SubsystemBase() {
        @Override
        public String getName() {
            return "Robot";
        }
    };
    private String[] cachedErrors = new String[0];
    private Notifier notifier;

    public HardwareMonitor() {
        devices.put(null, List.of(new Device() {
            @Override
            String getName() {
                return "RoboRIO";
            }

            @Override
            List<String> getErrors() {
                List<String> errors = new ArrayList<String>();
                if (RobotController.isBrownedOut()) {
                    errors.add("Browned Out");
                }
                if (RobotController.getFaultCount3V3() != 0) {
                    errors.add("3.3V Rail Faults: " + RobotController.getFaultCount3V3());
                }
                if (RobotController.getFaultCount5V() != 0) {
                    errors.add("5V Rail Faults: " + RobotController.getFaultCount5V());
                }
                if (RobotController.getFaultCount6V() != 0) {
                    errors.add("6V (PWM) Rail Faults: " + RobotController.getFaultCount6V());
                }
                return errors;
            }
        }));

        this.notifier = new Notifier(this::refresh);
        this.notifier.startPeriodic(1.0);
    }

    /** Represents a generic device that needs to be monitored for errors. */
    private static abstract class Device {
        Device() {
        }

        /** Return a "name" for the device. Can be CAN ID or a human readable name. */
        abstract String getName();

        /** Return the list of errors that this device is experiencing. */
        abstract List<String> getErrors();
    }

    /** Add a new device to the hardware monitor. */
    private void registerDevice(Subsystem subsystem, Device device) {
        if (subsystem == null) {
            subsystem = defaultSubsystem;
        }

        devices.putIfAbsent(subsystem, new ArrayList<Device>());
        devices.get(subsystem).add(device);
    }

    /** A `Device` specialized for the Spark Max motor controller. */
    private static class DeviceCANSparkBase extends Device {
        private CANSparkBase device;

        DeviceCANSparkBase(CANSparkBase device) {
            this.device = device;
        }

        @Override
        String getName() {
            if (device instanceof CANSparkFlex) {
                return "Spark Flex #" + device.getDeviceId();
            } else {
                return "Spark Max #" + device.getDeviceId();
            }
        }

        @Override
        List<String> getErrors() {
            List<String> errors = new ArrayList<String>();
            var faults = device.getFaults();

            if (faults != 0) {
                for (int bit = 0; bit < 15; bit++) {
                    if (((faults >> bit) & 1) != 0) {
                        errors.add(CANSparkBase.FaultID.fromId(bit).toString());
                    }
                }
            }

            var revError = device.getLastError();
            if (!revError.equals(REVLibError.kOk)) {
                errors.add(revError.toString());
            }

            return errors;
        }
    }

    /** Add a new device to the hardware monitor. */
    public void registerDevice(Subsystem subsystem, CANSparkBase device) {
        this.registerDevice(subsystem, new DeviceCANSparkBase(device));
    }

    /** A `Device` specialized for the TalonFX motor controller. */
    private static class DeviceTalonFX extends Device {
        private TalonFX device;
        private List<StatusSignal<Boolean>> faults;

        DeviceTalonFX(TalonFX device) {
            this.device = device;
            this.faults = List.of(
                    device.getFault_BootDuringEnable(),
                    device.getFault_BridgeBrownout(),
                    device.getFault_DeviceTemp(),
                    device.getFault_ForwardHardLimit(),
                    device.getFault_ForwardSoftLimit(),
                    device.getFault_FusedSensorOutOfSync(),
                    device.getFault_Hardware(),
                    device.getFault_MissingDifferentialFX(),
                    device.getFault_OverSupplyV(),
                    device.getFault_ProcTemp(),
                    device.getFault_RemoteSensorDataInvalid(),
                    device.getFault_RemoteSensorPosOverflow(),
                    device.getFault_RemoteSensorReset(),
                    device.getFault_ReverseHardLimit(),
                    device.getFault_ReverseSoftLimit(),
                    device.getFault_StatorCurrLimit(),
                    device.getFault_SupplyCurrLimit(),
                    device.getFault_Undervoltage(),
                    device.getFault_UnlicensedFeatureInUse(),
                    device.getFault_UnstableSupplyV(),
                    device.getFault_UsingFusedCANcoderWhileUnlicensed());
        }

        @Override
        String getName() {
            return "Talon FX #" + device.getDeviceID();
        }

        @Override
        List<String> getErrors() {
            List<String> errors = new ArrayList<String>();

            for (var fault : faults) {
                fault.refresh(true);
                if (fault.getValue()) {
                    errors.add(fault.getName());
                }
            }

            if (!BaseStatusSignal.isAllGood(faults.get(0))) {
                errors.add("CAN disconnected");
            }

            return errors;
        }
    }

    /** Add a new device to the hardware monitor. */
    public void registerDevice(Subsystem subsystem, TalonFX device) {
        this.registerDevice(subsystem, new DeviceTalonFX(device));
    }

    /** A `Device` specialized for the Pigeon2 IMU. */
    private static class DevicePigeon2 extends Device {
        private Pigeon2 device;
        private List<StatusSignal<Boolean>> faults;

        DevicePigeon2(Pigeon2 device) {
            this.device = device;
            this.faults = List.of(
                    device.getFault_BootDuringEnable(),
                    device.getFault_BootIntoMotion(),
                    device.getFault_BootupAccelerometer(),
                    device.getFault_BootupGyroscope(),
                    device.getFault_BootupMagnetometer(),
                    device.getFault_DataAcquiredLate(),
                    device.getFault_Hardware(),
                    device.getFault_LoopTimeSlow(),
                    device.getFault_SaturatedAccelerometer(),
                    device.getFault_SaturatedGyroscope(),
                    device.getFault_SaturatedMagnetometer(),
                    device.getFault_Undervoltage(),
                    device.getFault_UnlicensedFeatureInUse());
        }

        @Override
        String getName() {
            return "Pigeon 2 #" + device.getDeviceID();
        }

        @Override
        List<String> getErrors() {
            List<String> errors = new ArrayList<String>();

            for (var fault : faults) {
                fault.refresh(true);
                if (fault.getValue()) {
                    errors.add(fault.getName());
                }
            }

            if (!BaseStatusSignal.isAllGood(faults.get(0))) {
                errors.add("CAN disconnected");
            }

            return errors;
        }
    }

    /** Add a new device to the hardware monitor. */
    public void registerDevice(Subsystem subsystem, Pigeon2 device) {
        this.registerDevice(subsystem, new DevicePigeon2(device));
    }

    /** A `Device` specialized for the Power Distribution Hub. */
    private static class DevicePowerDistribution extends Device {
        private PowerDistribution device;

        DevicePowerDistribution(PowerDistribution device) {
            this.device = device;
        }

        @Override
        String getName() {
            if (device.getType() == ModuleType.kRev) {
                return "PDH #" + device.getModule();
            } else {
                return "PDP #" + device.getModule();
            }
        }

        @Override
        List<String> getErrors() {
            List<String> errors = new ArrayList<String>();

            PowerDistributionFaults faults = device.getFaults();
            if (faults.Brownout) {
                errors.add("Brownout");
            }

            if (faults.CanWarning) {
                errors.add("CAN Warning");
            }

            if (faults.HardwareFault) {
                errors.add("Hardware Fault");
            }

            for (int channel = 0; channel < 24; channel++) {
                if (Constants.kUpluggedPDH.contains(channel)) {
                    // Ignore PDH channels that are supposed to be unplugged.
                    continue;
                }

                if (faults.getBreakerFault(channel)) {
                    errors.add("Breaker #" + channel + " tripped");
                }
            }

            return errors;
        }
    }

    /** Add a new device to the hardware monitor. */
    public void registerDevice(Subsystem subsystem, PowerDistribution device) {
        this.registerDevice(subsystem, new DevicePowerDistribution(device));
    }

    /** Get the list of errors. */
    private String[] getErrors() {
        List<String> results = new ArrayList<String>();

        for (var entry : devices.entrySet()) {
            String subsystemName = "Robot";
            if (entry.getKey() != null) {
                subsystemName = entry.getKey().getName();
            }

            for (var device : entry.getValue()) {
                String deviceId = device.getName();
                for (var error : device.getErrors()) {
                    results.add(subsystemName + "[" + deviceId + "]: " + error);
                }
            }
        }

        return results.toArray(new String[results.size()]);
    }

    /** Poll the hardware to refresh the error list. */
    private void refresh() {
        String[] errors = getErrors();

        synchronized (this.cachedErrors) {
            this.cachedErrors = errors;
        }
    }

    /** Get one line of the error list. */
    public String getErrorLine(int i) {
        synchronized (this.cachedErrors) {
            if (i < cachedErrors.length) {
                return cachedErrors[i];
            }

            return "";
        }
    }

    @Override
    public void close() throws Exception {
        this.notifier.stop();
        this.notifier.close();
    }
}
