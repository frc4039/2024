package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BuildConstants;

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
}