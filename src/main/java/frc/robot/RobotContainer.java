// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpShootCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ClimbOnStageCommand;
import frc.robot.commands.EjectNoteCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeBeamBreakOverrideCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.IntakeNoteRumbleCommandGroup;
import frc.robot.commands.PivotAngleCommand;
import frc.robot.commands.PivotToShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpeakerShootParallelCommandGroup;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Helpers;

public class RobotContainer {
    // Create the "Main" tab first so it will be first in the list.
    public final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotAngleSubsystem pivotAngleSubsystem = new PivotAngleSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Create the "About" tab last so it will be last in the list.
    public final ShuffleboardTab aboutTab = Shuffleboard.getTab("About");

    private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
    private final Joystick m_operatorController = new Joystick(OperatorConstants.kOperatorControllerPort);

    private final JoystickButton driverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    private final JoystickButton driverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    private final JoystickButton driverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    private final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);

    private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController
            .getRawAxis(XboxController.Axis.kLeftTrigger.value) > OIConstants.kTriggerThreshold);
    private final Trigger driverRightTrigger = new Trigger(() -> m_driverController
            .getRawAxis(XboxController.Axis.kRightTrigger.value) > OIConstants.kTriggerThreshold);

    private final JoystickButton operatorRightBumper = new JoystickButton(m_operatorController,
            XboxController.Button.kRightBumper.value);

    private final JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController,
            XboxController.Button.kLeftBumper.value);

    private final Trigger operatorDLeftPadTrigger = new Trigger(() -> m_operatorController
            .getPOV() == 270);
    private final Trigger operatorDUpPadTrigger = new Trigger(() -> m_operatorController
            .getPOV() == 0);
    private final Trigger operatorDRightPadTrigger = new Trigger(() -> m_operatorController
            .getPOV() == 90);

    private final JoystickButton operatorBButton = new JoystickButton(m_operatorController,
            XboxController.Button.kB.value);

    private final JoystickButton operatorYButton = new JoystickButton(m_operatorController,
            XboxController.Button.kY.value);

    private final JoystickButton operatorAButton = new JoystickButton(m_operatorController,
            XboxController.Button.kA.value);

    private final JoystickButton operatorXButton = new JoystickButton(m_operatorController,
            XboxController.Button.kX.value);

    private final SendableChooser<Command> autoChooser;

    enum ScoringState {
        AMP,
        SPEAKER,
        CLIMB1,
        CLIMB2,
        CLIMB3
    }

    private ScoringState scoringState = ScoringState.SPEAKER;

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
                        OIConstants.kDriveDeadband),
                -1.0));
        // pivotAngleSubsystem.setDefaultCommand(new
        // PivotToShootCommand(pivotAngleSubsystem, driveSubsystem));

        // Register Named Commands
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("FeederCommand", new IndexerCommand(indexerSubsystem));
        NamedCommands.registerCommand("IndexerCommand", new IndexerCommand(indexerSubsystem));
        NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("IntakeNoteCommand", new IntakeNoteCommand(intakeSubsystem, indexerSubsystem));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        mainTab.add("Auto Chooser", autoChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
        mainTab.add("Zero Angle",
                new InstantCommand(() -> {
                    Rotation2d resetAngle = Rotation2d.fromDegrees(0);
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        resetAngle = Rotation2d.fromDegrees(180);
                    }
                    Translation2d currentPosition = driveSubsystem.getPose().getTranslation();
                    driveSubsystem.resetOdometry(new Pose2d(currentPosition, resetAngle));
                    System.out.println("Ran!");
                }).withName("Reset Angle")
                        .ignoringDisable(true))
                .withPosition(0, 1);
        mainTab.addString("RobotState", () -> scoringState.toString())
                .withPosition(1, 1);

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
        // _______________OPERATOR BUTTONS_______________\\
        operatorRightBumper.whileTrue(new IntakeBeamBreakOverrideCommand(intakeSubsystem, indexerSubsystem));
        operatorLeftBumper.whileTrue(new EjectNoteCommand(intakeSubsystem, indexerSubsystem));
        operatorBButton.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.AMP));
        operatorYButton.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.SPEAKER));
        operatorDLeftPadTrigger.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.CLIMB1));
        operatorDUpPadTrigger.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.CLIMB2));
        operatorDRightPadTrigger.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.CLIMB3));
        operatorAButton.whileTrue(new PivotAngleCommand(pivotAngleSubsystem));
        operatorXButton.whileTrue(new ClimbOnStageCommand(climberSubsystem));

        // _______________DRIVER BUTTONS_______________\\
        driverLeftTrigger.whileTrue(
                new SelectCommand<ScoringState>(Map.of(
                        ScoringState.SPEAKER,
                        new SpeakerShootParallelCommandGroup(
                                driveSubsystem, shooterSubsystem, indexerSubsystem, pivotAngleSubsystem,
                                () -> MathUtil.applyDeadband(
                                        m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                                        OIConstants.kDriveDeadband),
                                () -> MathUtil.applyDeadband(
                                        m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                                        OIConstants.kDriveDeadband)),
                        ScoringState.AMP, new AmpShootCommand(shooterSubsystem),
                        ScoringState.CLIMB1, new InstantCommand(),
                        ScoringState.CLIMB2, new InstantCommand(),
                        ScoringState.CLIMB3, new InstantCommand()), () -> scoringState));
        driverYButton.whileTrue(new TeleopDriveCommand(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
                        OIConstants.kDriveDeadband),
                0.0));
        driverBButton.whileTrue(new TeleopDriveCommand(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
                        OIConstants.kDriveDeadband),
                1.5 * Math.PI));
        driverAButton.whileTrue(new TeleopDriveCommand(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
                        OIConstants.kDriveDeadband),
                Math.PI));
        driverXButton.whileTrue(new TeleopDriveCommand(driveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                        OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
                        OIConstants.kDriveDeadband),
                Math.PI / 2));

        driverRightTrigger.whileTrue((new IndexerCommand(indexerSubsystem)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
