// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.jni.AprilTagJNI.Helper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagPracticeCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private double xSpeed;
    private boolean stopDriving;
    private ProfiledPIDController xController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);
    private ProfiledPIDController yController = new ProfiledPIDController(DriveConstants.kAimP,
            DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);
    Pose3d robotToTag;
    double robotX;
    double robotZ;
    double robotYaw;
    double tagRelativeYaw;
    double rotateSpeed;

    /** Creates a new DriveToNoteCommand. */
    public AprilTagPracticeCommand(DriveSubsystem driveSubsystem, double xSpeed) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController.reset(0, 0);
        yController.reset(0, 0);
        xController.setGoal(0);
        yController.setGoal(10.5);
        stopDriving = false;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /*
         * LimelightHelpers.setPipelineIndex("", 0);
         * double tx = LimelightHelpers.getTX("");
         * double ty = LimelightHelpers.getTY("");
         * boolean hasTarget = LimelightHelpers.getTV("");
         */
        robotToTag = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
        robotX = robotToTag.getX();
        robotZ = robotToTag.getZ() + 0.2;
        tagRelativeYaw = robotToTag.getRotation().getY();
        double subtractedYaw = robotYaw - tagRelativeYaw;

        robotToTag = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
        robotYaw = robotToTag.getRotation().getZ();

        System.out.println("X:" + robotX);
        System.out.println("Z:" + robotZ);
        System.out.println("Yaw:" + robotYaw);
        System.out.println("Subtracted Yaw:" + subtractedYaw);

        double xSpeed = (robotX / (robotX + robotZ)) * 0.5;
        double zSpeed = -1.0 * (robotZ / (robotX + robotZ)) * 0.5;
        double xSpeedRot = xSpeed * Math.cos(subtractedYaw) - zSpeed * Math.sin(subtractedYaw);
        double zSpeedRot = xSpeed * Math.sin(subtractedYaw) + zSpeed * Math.cos(subtractedYaw);
        if (robotYaw > Units.degreesToRadians(1)) {
            rotateSpeed = Math.signum(robotYaw) * 0.05;
        } else {
            rotateSpeed = 0;
        }

        driveSubsystem.drive(zSpeedRot, xSpeedRot, rotateSpeed, false, true);

        // if (hasTarget) {
        // driveSubsystem.drive(xController.calculate(tx), yController.calculate(ty), 0,
        // false, true);
        // }
        // if (tx > 0.5) {
        // driveSubsystem.drive(0, 0.07, 0, false, true);
        // }
        // if (tx < -0.5) {
        // driveSubsystem.drive(0, -0.07, 0, false, true);
        // }
        // if (tx < 0.5 && tx > -0.5) {
        // driveSubsystem.drive(0, 0, 0, false, true);
        // System.out.println("centered");
        // }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.sqrt(robotX * robotX + robotZ * robotZ) < 0.2) {
            return true;
        } else {
            return false;
        }
    }

    /*
     * public static double getTx() {
     * LimelightHelpers.setPipelineIndex("", 0);
     * double tx = LimelightHelpers.getTX("");
     * return tx;
     * }
     * 
     * public static double getTy() {
     * LimelightHelpers.setPipelineIndex("", 0);
     * double ty = LimelightHelpers.getTY("");
     * return ty;
     * }
     * 
     * public static boolean getTv() {
     * LimelightHelpers.setPipelineIndex("", 0);
     * boolean hasTarget = LimelightHelpers.getTV("");
     * return hasTarget;
     * }
     */
}
