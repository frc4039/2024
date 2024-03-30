// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.SensorConstants;

/** Add your docs here. */
public class Sensors {
    private static DigitalInput m_kBeamBreakDIO = new DigitalInput(SensorConstants.kBeamBreakDIO);
    private static DigitalInput m_kBeamBreakLowerDIO = Helpers.isBabycakes() ? m_kBeamBreakDIO
            : new DigitalInput(SensorConstants.kBeamBreakLowerDIO);

    public static boolean BeamBreakerIsBroken() {
        return !m_kBeamBreakDIO.get();
    }

    // Lower beam breaker is not installed on the robot.
    public static boolean LowerBeamBreakerIsBroken() {
        return !m_kBeamBreakLowerDIO.get();
    }

    public static double GetLimeLightCounter(String tableName) {
        NetworkTable photonVisionTable = NetworkTableInstance.getDefault().getTable("photonvision");
        return photonVisionTable.getSubTable(tableName)
                .getEntry("hearbeat").getDouble(0);
    }
}
