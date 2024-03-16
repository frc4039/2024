// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class WheelDiameterCalibrationCommand extends Command {
    private static final double driveRadius = DriveConstants.kDriveBaseRadius;

    private final DriveSubsystem drive;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelDiameterCalibrationCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = Math.toRadians(drive.getHeading());
        accumGyroYawRads = 0.0;

        drive.setWheelRadiusCailbration();
        startWheelPositions = drive.getSwerveModulePositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drive.drive(0.0, 0.0, 0.2, true, true);

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(Math.toRadians(drive.getHeading()) - lastGyroYawRads);
        lastGyroYawRads = Math.toRadians(drive.getHeading());
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = drive.getSwerveModulePositions();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        SmartDashboard.putNumber("wheel diameter", currentEffectiveWheelRadius);
    }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}