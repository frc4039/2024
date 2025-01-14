package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BuildConstants;
import frc.robot.LimelightHelpers;

public class Helpers {
    public static boolean isBabycakes() {
        return RobotController.getComments().equals("babycakes");
    }

    public static String getRobotName() {
        return RobotController.getComments();
    }

    public static String getGitBranch() {
        String dirtyString = " (dirty)";
        switch (BuildConstants.DIRTY) {
            case 0:
                dirtyString = "";
        }

        return BuildConstants.GIT_BRANCH + dirtyString;
    }

    public static double getTx() {
        LimelightHelpers.setPipelineIndex("", 0);
        double tx = LimelightHelpers.getTX("");
        return tx;
    }

    public static double getTy() {
        LimelightHelpers.setPipelineIndex("", 0);
        double ty = LimelightHelpers.getTY("");
        return ty;
    }

    public static boolean getTv() {
        LimelightHelpers.setPipelineIndex("", 0);
        boolean hasTarget = LimelightHelpers.getTV("");
        return hasTarget;
    }
}