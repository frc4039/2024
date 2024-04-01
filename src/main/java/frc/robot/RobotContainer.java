// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ScoringState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StageSide;
import frc.robot.commands.ActivateTrapCommand;
import frc.robot.commands.AdjustClimbAnalogLeftTriggerCommand;
import frc.robot.commands.AdjustClimbAnalogRightTriggerCommand;
import frc.robot.commands.AmpScoreCommand;
import frc.robot.commands.AutoDriveToNoteParallelRaceGroup;
import frc.robot.commands.AutoPreSpinIntake;
import frc.robot.commands.AutoPreSpinShooter;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoSubwooferShotSequentialCommandGroup;
import frc.robot.commands.ClimbOnStageCommand;
import frc.robot.commands.DriveToNoteCommand;
import frc.robot.commands.EjectNoteCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeBeamBreakOverrideCommand;
import frc.robot.commands.IntakeIndexShootCommandGroup;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.IntakeNoteRumbleCommandGroup;
import frc.robot.commands.LeftRobotCentricDriveCommand;
import frc.robot.commands.PivotAngleCommand;
import frc.robot.commands.PivotToClimbCommand;
import frc.robot.commands.PivotToShootCommand;
import frc.robot.commands.PivotToTravelCommand;
import frc.robot.commands.PodiumShooterCommand;
import frc.robot.commands.PreSpinShooter;
import frc.robot.commands.ReverseRobotCentricDriveCommand;
import frc.robot.commands.RightRobotCentricDriveCommand;
import frc.robot.commands.RobotCentricDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShuttleOverStageCommand;
import frc.robot.commands.ShuttleShootCommand;
import frc.robot.commands.SpeakerShootParallelCommandGroup;
import frc.robot.commands.SubwooferShootCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.WheelDiameterCalibrationCommand;
import frc.robot.subsystems.ActivateTrapSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;
import frc.robot.utils.MultiButtonTrigger;
import frc.robot.utils.Sensors;

public class RobotContainer {
    // Create the "Main" tab first so it will be first in the list.
    public final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    // Monitor for hardware faults that will display on the dashboard.
    private HardwareMonitor hardwareMonitor = new HardwareMonitor();

    // The robot's subsystems and commands are defined here...
    private ScoringState scoringState = ScoringState.SHUTTLE;

    // We need to keep the BlinkinSubsystem reference here- even though it's not
    // used in RobotContainer so that the subsystem gets loaded into memory for its
    // periodic() to run and operate the LED's.
    // The suppress warnings line applies only the the ony line after, and removes
    // the visual clue of having a potential error that's actually not an error, to
    // improve code readability.
    @SuppressWarnings("unused")
    private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem(() -> scoringState);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMonitor);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hardwareMonitor);
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem(hardwareMonitor);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMonitor);
    private final PivotAngleSubsystem pivotAngleSubsystem = new PivotAngleSubsystem(hardwareMonitor);
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(hardwareMonitor);
    private final ActivateTrapSubsystem TrapSubsystem = new ActivateTrapSubsystem(hardwareMonitor);

    // Create the "About" tab last so it will be last in the list.
    public final ShuffleboardTab aboutTab = Shuffleboard.getTab("About");

    private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
    private final Joystick m_operatorController = new Joystick(OperatorConstants.kOperatorControllerPort);

    private final JoystickButton driverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    private final JoystickButton driverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    private final JoystickButton driverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    private final JoystickButton driverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);

    private final JoystickButton driverStartButton = new JoystickButton(m_driverController,
            XboxController.Button.kStart.value);
    private final JoystickButton driverBackButton = new JoystickButton(m_driverController,
            XboxController.Button.kBack.value);

    private final JoystickButton operatorBackButton = new JoystickButton(m_operatorController,
            XboxController.Button.kBack.value);
    private final JoystickButton operatorStartButton = new JoystickButton(m_operatorController,
            XboxController.Button.kStart.value);
    private final MultiButtonTrigger climberTrigger = new MultiButtonTrigger(operatorBackButton, operatorStartButton);

    private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController
            .getRawAxis(XboxController.Axis.kLeftTrigger.value) > OIConstants.kTriggerThreshold);
    private final Trigger driverRightTrigger = new Trigger(() -> m_driverController
            .getRawAxis(XboxController.Axis.kRightTrigger.value) > OIConstants.kTriggerThreshold);
    private final Trigger operatorLeftTrigger = new Trigger(() -> m_operatorController
            .getRawAxis(XboxController.Axis.kLeftTrigger.value) > OIConstants.kTriggerThreshold);
    private final Trigger operatorRightTrigger = new Trigger(() -> m_operatorController
            .getRawAxis(XboxController.Axis.kRightTrigger.value) > OIConstants.kTriggerThreshold);

    private final DoubleSupplier driverLeftStickY = () -> MathUtil.applyDeadband(
            m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
            OIConstants.kDriveDeadband);
    private final DoubleSupplier driverLeftStickX = () -> MathUtil.applyDeadband(
            m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
            OIConstants.kDriveDeadband);
    private final DoubleSupplier driverRightStickX = () -> MathUtil.applyDeadband(
            m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
            OIConstants.kDriveDeadband);
    private final DoubleSupplier operatorRightStickX = () -> MathUtil.applyDeadband(m_operatorController.getRawAxis(
            XboxController.Axis.kRightX.value), OIConstants.kDriveDeadband);
    private final DoubleSupplier operatorLeftTriggerSupplier = () -> MathUtil.applyDeadband(
            m_operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value), OIConstants.kTriggerThreshold);
    private final DoubleSupplier operatorRightTriggerSupplier = () -> MathUtil.applyDeadband(
            m_operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value), OIConstants.kTriggerThreshold);

    private final JoystickButton operatorRightBumper = new JoystickButton(m_operatorController,
            XboxController.Button.kRightBumper.value);

    private final JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController,
            XboxController.Button.kLeftBumper.value);

    private final Trigger operatorDLeftPadTrigger = new Trigger(() -> m_operatorController
            .getPOV() == 270);
    private final Trigger operatorDUpPadTrigger = new Trigger(() -> m_operatorController
            .getPOV() == 0);
    private final Trigger operatorDDownPadTrigger = new Trigger(() -> m_operatorController
            .getPOV() == 180);
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

    private final JoystickButton driverRightBumper = new JoystickButton(m_driverController,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLeftBumper = new JoystickButton(m_driverController,
            XboxController.Button.kLeftBumper.value);

    private final Trigger driverDPadUpTrigger = new Trigger(() -> m_driverController.getPOV() == 0);
    private final Trigger driverDDownPadTrigger = new Trigger(() -> m_driverController.getPOV() == 180);
    private final Trigger driverDPadLeftTrigger = new Trigger(() -> m_driverController.getPOV() == 270);
    private final Trigger driverDPadRightTrigger = new Trigger(() -> m_driverController.getPOV() == 90);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Command> testSelector = new SendableChooser<Command>();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem,
                driverLeftStickY, driverLeftStickX, driverRightStickX, -1.0));
        shooterSubsystem.setDefaultCommand(new PreSpinShooter(shooterSubsystem, indexerSubsystem, () -> scoringState));
        // pivotAngleSubsystem.setDefaultCommand(new
        // PivotToShootCommand(pivotAngleSubsystem, driveSubsystem));

        // Register Named Commands
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("FeederCommand",
                new IndexerCommand(indexerSubsystem, shooterSubsystem, ShooterConstants.kShooterRPM - 200));
        NamedCommands.registerCommand("IndexerCommand",
                new IndexerCommand(indexerSubsystem, shooterSubsystem, ShooterConstants.kShooterRPM - 200));
        NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("IntakeNoteCommand", new IntakeNoteCommand(intakeSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("StartPivot",
                new ScheduleCommand(new PivotToShootCommand(pivotAngleSubsystem, driveSubsystem)));
        NamedCommands.registerCommand("TravelPivot",
                new ScheduleCommand(new PivotToTravelCommand(pivotAngleSubsystem)));
        NamedCommands.registerCommand("IntakeIndexShootCommand", new IntakeIndexShootCommandGroup(shooterSubsystem,
                indexerSubsystem, intakeSubsystem, m_driverController, m_operatorController));
        NamedCommands.registerCommand("SubwooferShot", new AutoSubwooferShotSequentialCommandGroup(driveSubsystem,
                shooterSubsystem, indexerSubsystem, pivotAngleSubsystem));
        NamedCommands.registerCommand("CalibrateWheelDiameter", new WheelDiameterCalibrationCommand(driveSubsystem));

        NamedCommands.registerCommand("AutoDriveToNotePRG",
                new AutoDriveToNoteParallelRaceGroup(intakeSubsystem, indexerSubsystem,
                        driveSubsystem));
        NamedCommands.registerCommand("AutoPreSpinShooter", new AutoPreSpinShooter(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("AutoPreSpinIntake", new AutoPreSpinIntake(intakeSubsystem));

        // Register Source876Blue Auto Conditional Commands for pathplanner Autos
        NamedCommands.registerCommand("zSource876BlueStep2",
                new ConditionalCommand(AutoBuilder.buildAuto("zSource876Blue2A"),
                        AutoBuilder.buildAuto("zSource876Blue2B"),
                        () -> indexerSubsystem.hasNote()));
        NamedCommands.registerCommand("zSource876BlueStep3",
                new ConditionalCommand(AutoBuilder.buildAuto("zSource876Blue3A"),
                        AutoBuilder.buildAuto("zSource876Blue3B"),
                        () -> indexerSubsystem.hasNote()));
        NamedCommands.registerCommand("zSource876BlueStep4",
                new ConditionalCommand(AutoBuilder.buildAuto("zSource876Blue4A"),
                        AutoBuilder.buildAuto("zSource876Blue4B"),
                        () -> indexerSubsystem.hasNote()));

        // Register Source876Red Auto Conditional Commands for pathplanner Autos
        NamedCommands.registerCommand("zSource876RedStep2",
                new ConditionalCommand(AutoBuilder.buildAuto("zSource876Red2A"),
                        AutoBuilder.buildAuto("zSource876Red2B"),
                        () -> indexerSubsystem.hasNote()));
        NamedCommands.registerCommand("zSource876RedStep3",
                new ConditionalCommand(AutoBuilder.buildAuto("zSource876Red3A"),
                        AutoBuilder.buildAuto("zSource876Red3B"),
                        () -> indexerSubsystem.hasNote()));
        NamedCommands.registerCommand("zSource876RedStep4",
                new ConditionalCommand(AutoBuilder.buildAuto("zSource876Red4A"),
                        AutoBuilder.buildAuto("zSource876Red4B"),
                        () -> indexerSubsystem.hasNote()));

        // Register Speaker3216 Blue Auto Conditional Commands for pathplanner Autos
        NamedCommands.registerCommand("zSpeaker3216BlueStep2",
                new ConditionalCommand(AutoBuilder.buildAuto("zSpeaker3216Blue2A"),
                        AutoBuilder.buildAuto("zSpeaker3216Blue2B"),
                        () -> indexerSubsystem.hasNote()));

        // Register Speaker3216 Red Auto Conditional Commands for pathplanner Autos
        NamedCommands.registerCommand("zSpeaker3216RedStep2",
                new ConditionalCommand(AutoBuilder.buildAuto("zSpeaker3216Red2A"),
                        AutoBuilder.buildAuto("zSpeaker3216Red2B"),
                        () -> indexerSubsystem.hasNote()));

        // Register Amp 145 Red Auto Conditional Commands for pathplanner Autos
        NamedCommands.registerCommand("zAmp145BlueStep2",
                new ConditionalCommand(AutoBuilder.buildAuto("zAmp145Blue2A"),
                        AutoBuilder.buildAuto("zAmp145Blue2B"),
                        () -> indexerSubsystem.hasNote()));
        NamedCommands.registerCommand("zAmp145BlueStep3",
                new ConditionalCommand(AutoBuilder.buildAuto("zAmp145Blue3A"),
                        AutoBuilder.buildAuto("zAmp145Blue3B"),
                        () -> indexerSubsystem.hasNote()));

        // Register Amp 145 Red Auto Conditional Commands for pathplanner Autos
        NamedCommands.registerCommand("zAmp145RedStep2",
                new ConditionalCommand(AutoBuilder.buildAuto("zAmp145Red2A"),
                        AutoBuilder.buildAuto("zAmp145Red2B"),
                        () -> indexerSubsystem.hasNote()));
        NamedCommands.registerCommand("zAmp145RedStep3",
                new ConditionalCommand(AutoBuilder.buildAuto("zAmp145Red3A"),
                        AutoBuilder.buildAuto("zAmp145Red3B"),
                        () -> indexerSubsystem.hasNote()));

        configureBindings();

        // autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("SPEAKER 3216 Blue Smart", new PathPlannerAuto("SPEAKER 3216 Blue Smart"));
        autoChooser.addOption("SPEAKER 3216 Red Smart", new PathPlannerAuto("SPEAKER 3216 Red Smart"));

        autoChooser.addOption("SOURCE 876 Blue Smart", new PathPlannerAuto("SOURCE 876 Blue Smart"));
        autoChooser.addOption("SOURCE 876 Red Smart", new PathPlannerAuto("SOURCE 876 Red Smart"));

        autoChooser.addOption("SOURCE 76 Blue Smart", new PathPlannerAuto("SOURCE 76 Blue Smart"));
        autoChooser.addOption("SOURCE 76 Red Smart", new PathPlannerAuto("SOURCE 76 Red Smart"));

        autoChooser.addOption("AMP 145 Blue Smart", new PathPlannerAuto("AMP 145 Blue Smart"));
        autoChooser.addOption("AMP 145 Red Smart", new PathPlannerAuto("AMP 145 Red Smart"));

        autoChooser.addOption("Disrupt Centre Auto", new PathPlannerAuto("Disrupt Centre Auto"));
        autoChooser.addOption("Wheel Calibration", new PathPlannerAuto("wheel calibration"));

        autoChooser.addOption("SPEAKER 3216 Blue", new PathPlannerAuto("SPEAKER 3216 Blue"));
        autoChooser.addOption("SPEAKER 3216 Red", new PathPlannerAuto("SPEAKER 3216 Red"));
        autoChooser.addOption("SOURCE 876 Blue", new PathPlannerAuto("SOURCE 876 Blue"));
        autoChooser.addOption("SOURCE 876 Red", new PathPlannerAuto("SOURCE 876 Red"));
        autoChooser.addOption("SOURCE 76 Blue", new PathPlannerAuto("SOURCE 76 Blue"));
        autoChooser.addOption("SOURCE 76 Red", new PathPlannerAuto("SOURCE 76 Red"));
        autoChooser.addOption("AMP 145 Blue", new PathPlannerAuto("AMP 145 Blue"));
        autoChooser.addOption("AMP 145 Red", new PathPlannerAuto("AMP 145 Red"));

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
                }).withName("Reset Angle")
                        .ignoringDisable(true))
                .withPosition(0, 1);
        mainTab.addString("RobotState", () -> scoringState.toString())
                .withPosition(1, 1);
        mainTab.addDouble("Pi Counter", () -> driveSubsystem.getPiCounter()).withPosition(0, 2);
        mainTab.addBoolean("Has Note", () -> Sensors.BeamBreakerIsBroken()).withPosition(1, 2);
        mainTab.addCamera("Note Cam", "NoteFeed",
                "mjpg:http://wpilibpi.local:1182/?action=stream")
                .withProperties(Map.of("showControls", false))
                .withPosition(2, 0)
                .withSize(3, 3);
        ShuffleboardLayout hardwareLayout = mainTab.getLayout("Hardware Errors", BuiltInLayouts.kList)
                .withPosition(2, 3)
                .withSize(3, 3)
                .withProperties(Map.of("Label position", "HIDDEN"));
        for (int i = 0; i < 10; i++) {
            final Integer index = i;
            hardwareLayout.addString(index.toString(), () -> hardwareMonitor.getErrorLine(9 - index));
        }

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
        robotInfo.addBoolean("Is Babycakes", () -> Helpers.isBabycakes());

        testSelector.setDefaultOption("None", new InstantCommand());
        SysIdRoutine pivotId = pivotAngleSubsystem.getSysId();
        testSelector.addOption("Pivot > QuasistaticForward", pivotId.quasistatic(Direction.kForward));
        testSelector.addOption("Pivot > QuasistaticReverse", pivotId.quasistatic(Direction.kReverse));
        testSelector.addOption("Pivot > DynamicForward", pivotId.dynamic(Direction.kForward));
        testSelector.addOption("Pivot > DynamicReverse", pivotId.dynamic(Direction.kReverse));

        aboutTab.add("Test Selector (driverStartButton)", testSelector)
                .withPosition(0, 2)
                .withSize(3, 1);
    }

    private void configureBindings() {
        // _______________OPERATOR BUTTONS_______________\\
        operatorDRightPadTrigger.whileTrue(
                new ConditionalCommand(
                        new ClimbOnStageCommand(climberSubsystem, ClimberConstants.kClimberMotorSpeed,
                                operatorRightStickX),
                        new IntakeBeamBreakOverrideCommand(intakeSubsystem, indexerSubsystem),
                        () -> this.scoringState == ScoringState.CLIMB));
        operatorDLeftPadTrigger.whileTrue(
                new ConditionalCommand(
                        new ClimbOnStageCommand(climberSubsystem, -ClimberConstants.kClimberMotorSpeed,
                                operatorRightStickX),
                        new EjectNoteCommand(intakeSubsystem, indexerSubsystem),
                        () -> this.scoringState == ScoringState.CLIMB));
        operatorBButton.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.LOW));
        operatorYButton.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.HIGH));
        operatorRightBumper.whileTrue(new IntakeBeamBreakOverrideCommand(intakeSubsystem, indexerSubsystem));
        operatorLeftBumper.whileTrue(new EjectNoteCommand(intakeSubsystem, indexerSubsystem));
        operatorBButton.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.LOW));
        operatorYButton.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.HIGH));
        climberTrigger.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.CLIMB));
        operatorDUpPadTrigger.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.SubwooferShoot));
        operatorDDownPadTrigger.onTrue(new InstantCommand(() -> this.scoringState = ScoringState.PodiumShoot));
        operatorAButton.whileTrue(new PivotAngleCommand(pivotAngleSubsystem,
                PivotConstants.kPivotAmpPosition)
        // .alongWith(new AmpShootCommand(shooterSubsystem)));
        // operatorXButton.onTrue(new InstantCommand(() -> this.scoringState =
        // ScoringState.INTAKE)
        );
        operatorRightTrigger.whileTrue(new ConditionalCommand(
                new AdjustClimbAnalogRightTriggerCommand(climberSubsystem, operatorRightTriggerSupplier),
                new InstantCommand(),
                () -> this.scoringState == ScoringState.CLIMB));

        operatorRightTrigger.onTrue(new ConditionalCommand(
                new InstantCommand(() -> this.scoringState = ScoringState.SHUTTLE),
                new InstantCommand(),
                () -> this.scoringState != ScoringState.CLIMB));
        operatorLeftTrigger
                .whileTrue(new ConditionalCommand(
                        new AdjustClimbAnalogLeftTriggerCommand(climberSubsystem, operatorLeftTriggerSupplier),
                        new IntakeNoteRumbleCommandGroup(intakeSubsystem, indexerSubsystem,
                                m_driverController, m_operatorController),
                        () -> this.scoringState == ScoringState.CLIMB));
        operatorXButton.whileTrue(
                new ConditionalCommand(new ActivateTrapCommand(TrapSubsystem), new InstantCommand(),
                        () -> this.scoringState == ScoringState.CLIMB));

        // _______________DRIVER BUTTONS_______________\\
        driverLeftTrigger.whileTrue(
                new SelectCommand<ScoringState>(Map.of(
                        ScoringState.HIGH,
                        new SpeakerShootParallelCommandGroup(
                                driveSubsystem, shooterSubsystem, indexerSubsystem, pivotAngleSubsystem,
                                driverLeftStickY, driverLeftStickX),
                        ScoringState.LOW,
                        new TeleopDriveCommand(driveSubsystem,
                                driverLeftStickY, driverLeftStickX, driverRightStickX, 0.5 * Math.PI),
                        ScoringState.INTAKE,
                        new DriveToNoteCommand(driveSubsystem, indexerSubsystem, DriveConstants.kDriveToNoteXSpeed),
                        ScoringState.SubwooferShoot,
                        new PivotAngleCommand(pivotAngleSubsystem, PivotConstants.kPivotSubwooferPosition)
                                .alongWith(new SubwooferShootCommand(shooterSubsystem)
                                        .alongWith(new TeleopDriveCommand(driveSubsystem,
                                                driverLeftStickY, driverLeftStickX, driverRightStickX, -1.0))),
                        ScoringState.PodiumShoot,
                        new PivotAngleCommand(pivotAngleSubsystem, PivotConstants.kPivotPodiumPosition)
                                .alongWith(new PodiumShooterCommand(shooterSubsystem)
                                        .alongWith(new TeleopDriveCommand(driveSubsystem,
                                                driverLeftStickY, driverLeftStickX, driverRightStickX, -1.0))),
                        ScoringState.SHUTTLE,
                        new PivotAngleCommand(pivotAngleSubsystem, PivotConstants.kPivotShuttleOverStage)
                                .alongWith(new ShuttleOverStageCommand(shooterSubsystem))
                                .alongWith(new TeleopDriveCommand(driveSubsystem, driverLeftStickY, driverLeftStickX,
                                        driverRightStickX,

                                        () -> {
                                            Optional<Alliance> currentAlliance = DriverStation.getAlliance();
                                            if (currentAlliance.isPresent()) {
                                                switch (currentAlliance.get()) {
                                                    case Red:
                                                        return ShooterConstants.kShuttleOverStageYawRed;
                                                    case Blue:
                                                        return ShooterConstants.kShuttleOverStageYawBlue;
                                                    default:
                                                        return -1.0;
                                                }
                                            }
                                            return -1.0;
                                        }))),
                        // ScoringState.CLIMB,
                        // new TrapScoreCommand(pivotAngleSubsystem, shooterSubsystem,
                        // indexerSubsystem)),
                        () -> scoringState));

        // driverYButton.whileTrue(new AimAtNoteCommand(driveSubsystem,
        // driverLeftStickY, driverLeftStickX));

        driverYButton.whileTrue(
                new ConditionalCommand(new PivotToTravelCommand(pivotAngleSubsystem),
                        new DriveToNoteCommand(driveSubsystem, indexerSubsystem,
                                DriveConstants.kDriveToNoteXSpeed),
                        () -> this.scoringState == ScoringState.CLIMB));

        driverXButton.whileTrue(
                new TeleopDriveCommand(
                        driveSubsystem, driverLeftStickY, driverLeftStickX, driverRightStickX, StageSide.LEFT));
        driverBButton.whileTrue(
                new TeleopDriveCommand(
                        driveSubsystem, driverLeftStickY, driverLeftStickX, driverRightStickX, StageSide.RIGHT));
        driverAButton.onTrue(new ConditionalCommand(
                new PivotToClimbCommand(pivotAngleSubsystem, driveSubsystem, PivotConstants.kPivotTrapPosition)
                        .withTimeout(5.0),
                new InstantCommand(), () -> this.scoringState == ScoringState.CLIMB));

        driverRightTrigger.whileTrue(new SelectCommand<ScoringState>(Map.of(
                ScoringState.HIGH,
                new IndexerCommand(indexerSubsystem, shooterSubsystem, ShooterConstants.kShooterRPM - 200),
                ScoringState.SubwooferShoot,
                new IndexerCommand(indexerSubsystem, shooterSubsystem, ShooterConstants.kSubwooferShooterRPM - 200),
                ScoringState.PodiumShoot,
                new IndexerCommand(indexerSubsystem, shooterSubsystem, ShooterConstants.kPodiumShooterRPM - 200),
                ScoringState.SHUTTLE,
                new IndexerCommand(indexerSubsystem, shooterSubsystem, ShooterConstants.kShuttleShootRPM - 200)),
                () -> scoringState));

        driverRightBumper.whileTrue(
                new AmpScoreCommand(pivotAngleSubsystem, shooterSubsystem, indexerSubsystem));

        driverLeftBumper.whileTrue(AutoBuilder.pathfindThenFollowPath(
                AutoConstants.pathFindingAmpPath,
                AutoConstants.pathFindingConstraints,
                0.0));

        driverDPadUpTrigger.whileTrue(
                new ReverseRobotCentricDriveCommand(driveSubsystem));
        driverDDownPadTrigger.whileTrue(
                new RobotCentricDriveCommand(driveSubsystem));
        driverDPadLeftTrigger.whileTrue(
                new LeftRobotCentricDriveCommand(driveSubsystem));
        driverDPadRightTrigger.whileTrue(
                new RightRobotCentricDriveCommand(driveSubsystem));

        driverStartButton.whileTrue(new ShuttleShootCommand(shooterSubsystem, indexerSubsystem,
                () -> ShooterConstants.kTrapShooterRPM));

        driverBackButton
                .and(() -> DriverStation.isTestEnabled())
                .whileTrue(new ProxyCommand(() -> testSelector.getSelected()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
