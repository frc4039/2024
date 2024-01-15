// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OperatorConstants.kOperatorControllerPort);
  
  private DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new TeleopDrive(driveSubsystem,
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value), OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value), OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value), OIConstants.kDriveDeadband)
    ));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
