// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  // Create the "Main" tab first so it will be first in the list.
  public final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  // Create the "About" tab last so it will be last in the list.
  public final ShuffleboardTab aboutTab = Shuffleboard.getTab("About");

  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OperatorConstants.kOperatorControllerPort);

  private final JoystickButton driverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton driverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  
  private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController.getRawAxis(XboxController.Axis.kLeftTrigger.value) > OIConstants.kTriggerThreshold);
  private final JoystickButton driverRightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

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

  private void configureBindings() {
    driverRightBumper.whileTrue(new IntakeSpin(intakeSubsystem));
  
    driverLeftTrigger.whileTrue(new ShootCommand(() -> m_driverController.getRawAxis(XboxController.Axis.kLeftTrigger.value), shooter));
    driverYButton.whileTrue(new AmpShoot(shooter));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

