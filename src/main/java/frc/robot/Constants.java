// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Helpers;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Ports that are empty on the PDH that should not show as faults. */
    public static Set<Integer> kUpluggedPDH = Set.of(4, 6, 9);

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final boolean kPointToTurn = false;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 5.45; // was 5.45// dont let ben know that i can make this
                                                                    // higher
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second for turning

        public static final double kDirectionSlewRate = 8; // radians per second Higher is faster
        public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%) Higher is faster
        public static final double kRotationalSlewRate = 15; // percent per second (1 = 100%) Higher is faster
        public static final double kAimP = 1.7;
        public static final double kAimI = 0;
        public static final double kAimD = 0;
        public static final Constraints kAimProfile = new Constraints(3 * Math.PI, 2 * Math.PI);

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        public static final double kDriveBaseRadius = 15.9;// Math
        // .sqrt(Math.pow(kTrackWidth / 2, 2) + Math.pow(kWheelBase / 2, 2));
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // Pigeon CAN ID
        public static final int kGyroID = 20;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 13;
        public static final int kFrontRightDrivingCanId = 15;
        public static final int kRearRightDrivingCanId = 17;

        public static final int kFrontLeftTurningCanId = 10;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kFrontRightTurningCanId = 14;
        public static final int kRearRightTurningCanId = 16;

        public static final boolean kGyroReversed = false;

        public static final double kDriveToNoteXSpeed = 0.3;
        public static final double kDriveToNoteYSpeed = 0.0;

        public static final double kAutoDriveToNoteXSpeed = 0.3;
        public static final double kAutoDriveToNoteDistance = 1.0;
        public static final double kAutoDriveToNoteTime = 2.0;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRPM = 6380;
        public static final double kDrivingMotorFreeSpeedRps = kDrivingMotorFreeSpeedRPM / 60;
        public static final double kWheelDiameterInches = 2.91;
        public static final double kWheelDiameterMeters = kWheelDiameterInches * 0.0254;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion.
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static Double kDrivingRotationsToMeters = (1.0 / kDrivingMotorReduction) * kWheelCircumferenceMeters;
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
        public static final double kDriveWheelFreeSpeedFps = kDriveWheelFreeSpeedRps * 3.28084; // conversion from m/s
                                                                                                // to ft/s because i got
                                                                                                // tired of doing the
                                                                                                // conversion every time
                                                                                                // ben asked
        // might be able to remove drive speed math - will check at later date
        public static final double kMpsToPercentOutput = kDrivingMotorReduction * 60 / kWheelCircumferenceMeters
                / kDrivingMotorFreeSpeedRPM; // Multiply by meters per second to get percent output

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 30.0; // radians per second used to
                                                                                         // be /60

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.01;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0.01;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1.1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;// DO NOT INCREASE
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final NeutralModeValue kDrivingMotorNeutralMode = NeutralModeValue.Brake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
        public static final double kDrivingMotorNeutralDeadband = 0.005; // Jagon Baker did this

        public static final int kDrivingMotorCurrentLimit = 60; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        public static final int kDrivingMotorCurrentThreshhold = 50; // amps
        public static final double kTriggerThresholdTime = 1.5; // seconds
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kTriggerThreshold = 0.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class ShooterConstants {
        public static final int kLowerShooterCANId = 30;
        public static final int kUpperShooterCANId = 31;

        public static final double kAmpUpperMotorSpeed = -0.5;
        public static final double kAmpLowerMotorSpeed = 0.5;

        public static final double kHumanPlayerUpperMotorSpeed = -0.2;
        public static final double kHumanPlayerLowerMotorSpeed = -0.2;

        public static final double kShooterSpeedLimit = 0.7;
        public static final double kShooterAmpSpeedLimit = 0.7;
        public static final double kShooterHumanPlayerSpeedLimit = 0.7;

        public static final double kShooterP = 0.0005;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.001;
        public static final double kShooterFF = 0.000145;

        public static final double kShooterRPM = 4000;
        public static final double kAmpRPM = 3000;
        public static final double kTrapShooterRPM = 600;
        public static final double kSubwooferShooterRPM = 2750;
        public static final double kPodiumShooterRPM = 4000;
        public static final double kShuttleShootRPM = 1000;
    }

    public static final class IndexerConstants {
        public static final double kIndexerShooterSpeed = 0.8;
        public static final double kIndexerIntakeSpeed = 0.80;
        public static final double kIndexerHumanPlayerSpeed = -0.15;
        public static final DigitalInput kBeamBreakDIO = new DigitalInput(2);
        public static final DigitalInput kBeamBreakLowerDIO = Helpers.isBabycakes() ? kBeamBreakDIO
                : kBeamBreakDIO;// new DigitalInput(3); // Set this to channel 3 once this beam breaker is added
                                // to rumble
        public static final int kIndexerCANID = 32;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorCANID = 40;
        public static final double kIntakeSpeedMotor = 1;
    }

    public static final class PivotConstants {
        public static final int kPivotCANId = 50;
        public static final int kPivotFollowerCANId = 51;

        public static final double kPivotEncoderPositionFactor = 360.0;
        public static final double kPivotEncoderVelocityFactor = 360.0 / 60.0;

        public static final double kPivotP = 0.03;
        public static final double kPivotI = 0;
        public static final double kPivotD = 0.20;
        public static final double kPivotFF = 0;
        public static final double kPivotMinOutput = -0.5;
        public static final double kPivotMaxOutput = 0.5;

        // Offset should put 0 degrees straight down.
        // To calibrate, straight up should read 180 on the dashboard.
        // This value must be positive. Negative values do not work.
        public static final double kPivotOffset = Helpers.isBabycakes() ? 56 : 351.4;
        // Values on the encoder should move towards the shooter side.
        // Values >180 should be towards bellypan.
        // Values <180 should be to the amp / open side of the robot.
        public static final boolean kPivotEncoderInverted = false;

        public static final double kPivotTravelPosition = Helpers.isBabycakes() ? 237 : 251;
        public static final double kPivotAmpPosition = 162; // was 169 //before NM 159 163.5 162 = 20 degrees relative
                                                            // to vertical
        public static final double kPivotSubwooferPosition = 212;
        public static final double kPivotPodiumPosition = 235;
        public static final double kPivotTrapPosition = 170; // 162;

        public static final double kPivotAngleClose = Helpers.isBabycakes() ? 211.0 : 218.0;
        public static final double kPivotDistanceClose = Helpers.isBabycakes() ? 1.37 : 1.32;
        public static final double kPivotAngleMedium = Helpers.isBabycakes() ? 226.0 : 224.0;
        public static final double kPivotDistanceMedium = Helpers.isBabycakes() ? 3.0 : 1.57;
        public static final double kPivotAngleFar = Helpers.isBabycakes() ? 233.0 : 227.0;
        public static final double kPivotDistanceFar = Helpers.isBabycakes() ? 4.12 : 2.1;
        public static final double kPivotAngle4 = 233.0;
        public static final double kPivotDistance4 = 2.53;
        public static final double kPivotAngle5 = 237.0;
        public static final double kPivotDistance5 = 3.04;
        public static final double kPivotAngle6 = 240;
        public static final double kPivotDistance6 = 3.51;
        public static final double kPivotAngle7 = 243.0;
        public static final double kPivotDistance7 = 4.11;
        public static final double kPivotAngle8 = 245.0;
        public static final double kPivotDistance8 = 4.49;
        public static final double kPivotAngle9 = 247.0;
        public static final double kPivotDistance9 = 5.06;
    }

    public static final class ClimberConstants {
        public static final int kClimberLeaderMotorCANId = 55;
        public static final int kClimberFollowerMotorCANId = 56;
        public static final int kClimberSmartCurrentLimit = 20;
        public static final double kClimberMotorSpeed = 0.5; // setting to .1 to test direction of motors. 0.4;
    }

    public static class VisionConstants {
        public static final String kCameraRightBackName = "LimelightRightBack";
        public static final String kCameraLeftBackName = "LimelightLeftBack";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d kRobotToCamRightBack = new Transform3d(
                new Translation3d(0.23, 0.5, 0.201512),
                new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-26),
                        Units.degreesToRadians(166)));

        // Back camera mounted 11.0 inches behind centre, 8.5 left of centre, 8.625
        // inches up from centre, 24 degrees for horizontal

        // this camera is on both robots. So we need to change the position based on
        // which robot.

        public static final Transform3d kRobotToCamLeftBack = Helpers.isBabycakes()
                ? new Transform3d( // babycakes camera mounting transform
                        new Translation3d(Units.inchesToMeters(-11.0), Units.inchesToMeters(8.5),
                                Units.inchesToMeters(8.625)),
                        new Rotation3d(0, Units.degreesToRadians(-24),
                                Units.degreesToRadians(180.00)))
                : new Transform3d( // compBot camera mounting
                        new Translation3d(-0.45, -0.1, 0.201512),
                        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-26),
                                Units.degreesToRadians(183.5)));
        /*
         * public static final Transform3d kRobotToCamLeftBack = new Transform3d(
         * new Translation3d(0.236250, 0.274603, 0.201512),
         * new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-27.5),
         * Units.degreesToRadians(190.00)));// was 170
         */

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 1000);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class BlinkinConstants {
        public static final int kBlinkinPWMPort = 0;

        // Colours
        public static final double kColourValueGreen = 0.77;
        public static final double kColourValueGreenFlashing = -0.47;
        public static final double kColourValueRainbow = -0.89;
    }
}
