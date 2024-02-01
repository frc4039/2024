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

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.PivotAngleCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TurnToGamePiece;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Helpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;

public class RobotContainer {
    // Create the "Main" tab first so it will be first in the list.
    public final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotAngleSubsystem pivotAngleSubsystem = new PivotAngleSubsystem();

    // Create the "About" tab last so it will be last in the list.
    public final ShuffleboardTab aboutTab = Shuffleboard.getTab("About");

    private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
    private final Joystick m_operatorController = new Joystick(OperatorConstants.kOperatorControllerPort);

    private final JoystickButton driverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    private final JoystickButton driverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);

    private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController
            .getRawAxis(XboxController.Axis.kLeftTrigger.value) > OIConstants.kTriggerThreshold);
    private final JoystickButton driverRightBumper = new JoystickButton(m_driverController,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton driverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);

    private final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDrive(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
                        OIConstants.kDriveDeadband)));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        mainTab.add("Auto Chooser", autoChooser);

        aboutTab.addBoolean("Is Blackout", () -> Helpers.isBlackout());
        aboutTab.addString("Robot Comments", () -> Helpers.getRobotName());
    }

    private void configureBindings() {
        driverRightBumper.whileTrue(new IntakeNoteCommand(intakeSubsystem, feederSubsystem));
        driverXButton.whileTrue(new TurnToGamePiece(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband)));

        driverLeftTrigger.whileTrue(
                new ShootCommand(shooterSubsystem));
        driverYButton.whileTrue(new AmpShoot(shooterSubsystem, feederSubsystem));
        driverAButton.whileTrue((new FeederCommand(feederSubsystem)));
        driverBButton.whileTrue((new PivotAngleCommand(pivotAngleSubsystem)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
