// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TurnToGamePiece;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Helpers;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    // Create the "Main" tab first so it will be first in the list.
    public final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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

        ShuffleboardLayout buildInfo = aboutTab.getLayout("Build Info", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(3, 2)
                .withProperties(Map.of("Label position", "TOP"));
        buildInfo.add("Git Branch", Helpers.getGitBranch());
        buildInfo.add("Git SHA", BuildConstants.GIT_SHA);
        buildInfo.add("Build Date", BuildConstants.BUILD_DATE);

        ShuffleboardLayout robotInfo = aboutTab.getLayout("Robot Info", BuiltInLayouts.kList)
                .withPosition(3, 0)
                .withSize(3, 2)
                .withProperties(Map.of("Label position", "TOP"));
        robotInfo.addString("Robot Comments", () -> Helpers.getRobotName());
        robotInfo.addBoolean("Is Blackout", () -> Helpers.isBlackout());
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
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
