// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.Vision;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroID);

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Driver station position
    private Field2d fieldDisplay = new Field2d();

    // Odometry class for tracking robot pose
    // Pose estimator using odometry and april tags
    private Vision m_camRightBack;
    private Vision m_camLeftBack;
    private SwerveDrivePoseEstimator m_poseEstimator;

    private NetworkTable m_piVision;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(HardwareMonitor hw) {
        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(0.0),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                }, new Pose2d());

        if (!Helpers.isBabycakes()) {
            m_camRightBack = new Vision(VisionConstants.kCameraRightBackName, VisionConstants.kRobotToCamRightBack);
        }
        m_camLeftBack = new Vision(VisionConstants.kCameraLeftBackName, VisionConstants.kRobotToCamLeftBack);

        m_piVision = NetworkTableInstance.getDefault().getTable("PiVision");

        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
        driveTab.addDouble("X Meters", () -> getPose().getX())
                .withPosition(0, 0);
        driveTab.addDouble("Y Meters", () -> getPose().getY())
                .withPosition(0, 1);
        driveTab.addDouble("Angle", () -> getPose().getRotation().getDegrees())
                .withPosition(0, 2);
        driveTab.add("Subsystem", this)
                .withPosition(7, 0)
                .withSize(2, 1);
        driveTab.addDouble("Speaker Distance", () -> getTranslationToGoal().getNorm())
                .withPosition(4, 0)
                .withSize(1, 1);
        driveTab.addDouble("Note Angle", () -> getNoteAngle())
                .withPosition(4, 0)
                .withSize(1, 1);

        ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
        mainTab.add("Field", fieldDisplay)
                .withPosition(5, 0)
                .withSize(7, 4);

        m_frontLeft.registerWithHardwareTracker(this, hw);
        m_frontRight.registerWithHardwareTracker(this, hw);
        m_rearLeft.registerWithHardwareTracker(this, hw);
        m_rearRight.registerWithHardwareTracker(this, hw);
        hw.registerDevice(this, m_gyro);

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance. This will flip the path being followed to the red side of the
                    // field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        var gyroConfig = new MountPoseConfigs();
        gyroConfig.MountPoseYaw = 0;
        gyroConfig.MountPosePitch = 0;
        gyroConfig.MountPoseRoll = 0;
        m_gyro.getConfigurator().apply(gyroConfig);
        m_gyro.setYaw(0);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_poseEstimator.update(
                Rotation2d.fromDegrees(m_gyro.getYaw().getValue()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        // Insert vision logic here
        Optional<EstimatedRobotPose> estimatedGlobalPoseLeftBack = m_camLeftBack.getEstimatedGlobalPose();
        double numberOfTagsLeftBack = 0.0;
        if (estimatedGlobalPoseLeftBack.isPresent()) {
            EstimatedRobotPose camPoseLeftBack = estimatedGlobalPoseLeftBack.get();
            numberOfTagsLeftBack = m_camLeftBack.getNumberOfTags(camPoseLeftBack.estimatedPose.toPose2d());

            if (numberOfTagsLeftBack < 2.0) {
                Double ambiguityLeftBack = m_camLeftBack.getAmbiguity(camPoseLeftBack.estimatedPose.toPose2d());
                if (ambiguityLeftBack < 0.5) {
                    // check rotation compared to current heading. Accept if within threshold
                    Rotation2d currentRotation = getPose().getRotation(); // should be gyro directly?
                    if (Math.abs(currentRotation.minus(camPoseLeftBack.estimatedPose.getRotation().toRotation2d())
                            .getDegrees()) < 15) {
                        fieldDisplay.getObject("Camera Left Pose").setPose(camPoseLeftBack.estimatedPose.toPose2d());
                        m_poseEstimator.addVisionMeasurement(
                                camPoseLeftBack.estimatedPose.toPose2d(), camPoseLeftBack.timestampSeconds,
                                m_camRightBack.getEstimationStdDevs(camPoseLeftBack.estimatedPose.toPose2d()));
                    }
                }
            } else {
                fieldDisplay.getObject("Camera Left Pose").setPose(camPoseLeftBack.estimatedPose.toPose2d());
                m_poseEstimator.addVisionMeasurement(
                        camPoseLeftBack.estimatedPose.toPose2d(), camPoseLeftBack.timestampSeconds,
                        m_camRightBack.getEstimationStdDevs(camPoseLeftBack.estimatedPose.toPose2d()));
            }

        }
        SmartDashboard.putNumber("Left number of tags", numberOfTagsLeftBack);

        if (!Helpers.isBabycakes()) {
            Optional<EstimatedRobotPose> estimatedGlobalPoseRightBack = m_camRightBack
                    .getEstimatedGlobalPose();
            double numberOfTagsRightBack = 0.0;
            if (estimatedGlobalPoseRightBack.isPresent()) {
                EstimatedRobotPose camPoseRightBack = estimatedGlobalPoseRightBack.get();
                numberOfTagsRightBack = m_camRightBack.getNumberOfTags(camPoseRightBack.estimatedPose.toPose2d());

                if (numberOfTagsRightBack < 2.0) {
                    Double ambiguityRightBack = m_camRightBack.getAmbiguity(camPoseRightBack.estimatedPose.toPose2d());

                    if (ambiguityRightBack < 0.5) {
                        // check rotation compared to current heading. Accept if within threshold
                        Rotation2d currentRotation = getPose().getRotation();
                        if (Math.abs(currentRotation.minus(camPoseRightBack.estimatedPose.getRotation().toRotation2d())
                                .getDegrees()) < 15) {
                            fieldDisplay.getObject("Camera Right Pose")
                                    .setPose(camPoseRightBack.estimatedPose.toPose2d());
                            m_poseEstimator.addVisionMeasurement(
                                    camPoseRightBack.estimatedPose.toPose2d(), camPoseRightBack.timestampSeconds,
                                    m_camRightBack.getEstimationStdDevs(camPoseRightBack.estimatedPose.toPose2d()));
                        }
                    }
                } else {
                    fieldDisplay.getObject("Camera Right Pose").setPose(camPoseRightBack.estimatedPose.toPose2d());

                    m_poseEstimator.addVisionMeasurement(
                            camPoseRightBack.estimatedPose.toPose2d(), camPoseRightBack.timestampSeconds,
                            m_camRightBack.getEstimationStdDevs(camPoseRightBack.estimatedPose.toPose2d()));
                }

            }
            SmartDashboard.putNumber("Right number of tags", numberOfTagsRightBack);
        }
        fieldDisplay.setRobotPose(getPose());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
                Rotation2d.fromDegrees(m_gyro.getYaw().getValue()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        if (fieldRelative == true) {
            // Speed needs to always be away from the alliance wall.
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                xSpeedDelivered = -xSpeedDelivered;
                ySpeedDelivered = -ySpeedDelivered;
            }
        }

        driveRaw(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
    }

    /**
     * Drive with real units for the movement.
     */
    public void driveRaw(double xSpeedMetersPerSecond, double ySpeedMetersPerSecond, double rotRadiansPerSecond,
            boolean fieldRelative) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
                                rotRadiansPerSecond, getPose().getRotation())
                        : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotRadiansPerSecond));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setWheelRadiusCailbration() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public double[] getSwerveModulePositions() {
        return new double[] {
                m_frontLeft.getWheelRadians(),
                m_frontRight.getWheelRadians(),
                m_rearLeft.getWheelRadians(),
                m_rearRight.getWheelRadians()
        };
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return getPose().getRotation().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getAngularVelocityYDevice().getValue();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.driveRaw(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                false);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                new SwerveModuleState[] {
                        m_frontLeft.getState(),
                        m_frontRight.getState(),
                        m_rearLeft.getState(),
                        m_rearRight.getState()
                });
    }

    public Translation2d getTranslationToGoal() {
        var goalposition = new Translation2d(0, 5.55);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            goalposition = new Translation2d(16.46, 5.55);
        }
        return getPose().getTranslation().minus(goalposition);
    }

    public Translation2d getTranslationToCorner() {
        var cornerposition = new Translation2d(0, 8);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            cornerposition = new Translation2d(16.46, 8);
        }
        return getPose().getTranslation().minus(cornerposition);
    }

    /** Get the angle from the robot to the note */
    public double getNoteAngle() {
        return this.m_piVision.getEntry("Angle").getDouble(0);
    }

    public double getPiCounter() {
        return this.m_piVision.getEntry("ThreadCounter").getDouble(0);
    }

    /** Return if Camera detects a note */
    public Boolean getNoteDetected() {
        return this.m_piVision.getEntry("DetectedNote").getBoolean(false);
    }

    /** Return x Pose Value */
    public double getPoseXValue() {
        return this.getPose().getX();
    }
}