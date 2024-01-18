// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // Create the "Main" tab first so it will be first in the list.
  public final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  // The robot's subsystems and commands are defined here...
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  
  // Create the "About" tab last so it will be last in the list.
  public final ShuffleboardTab aboutTab = Shuffleboard.getTab("About");

  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OperatorConstants.kOperatorControllerPort);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationXAxis = XboxController.Axis.kRightX.value;
  private final int rotationYAxis = XboxController.Axis.kRightY.value;


  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new TeleopDrive(driveSubsystem,
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value), OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value), OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value), OIConstants.kDriveDeadband)
    ));
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    mainTab.add("Auto Chooser", autoChooser);
  }



  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}