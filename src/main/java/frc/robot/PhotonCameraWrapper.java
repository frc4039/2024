package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonCameraWrapper {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    public PhotonCameraWrapper(String cameraName, Transform3d cameraOffset) {
        camera = new PhotonCamera(cameraName);
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                cameraOffset);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (poseEstimator == null) {
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }
}
