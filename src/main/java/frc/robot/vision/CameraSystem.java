package frc.robot.vision;

import com.revrobotics.spark.config.LimitSwitchConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

public class CameraSystem {


    private static final double POSE_CUTOFF = 0.2;
    private AprilTagFieldLayout fieldLayout;
    private final List<Camera> cameras;

    public CameraSystem() {

        try {
            //TODO add apriltag layout
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            fieldLayout = null;
            System.err.println("April Tag Field Layout Failed to Load");
        }
        cameras = List.of(
                new Camera(new Translation3d(inchToMeter(7.75), inchToMeter(-4), inchToMeter(8.25)), new Rotation3d(Math.PI, -Math.PI / 4, 0), "alpha")
        );


    }

    public List<Pose3d> getCameraMeasurements() {

        LinkedList<Pose3d> results = new LinkedList<>();

        for (Camera camera : cameras) {

            for (PhotonPipelineResult result : camera.getCamera().getAllUnreadResults()) {

                if (result.hasTargets()) {

                    for (PhotonTrackedTarget target : result.getTargets()) {
                        if (target.getFiducialId() > -1 && target.getPoseAmbiguity() <= POSE_CUTOFF && target.getPoseAmbiguity() != -1) {

                            Pose3d robotPose = getRobotLocation(camera.getCameraToRobot(), target.getBestCameraToTarget(), target.getFiducialId());

                            results.add(robotPose);
                        }


                    }
                }

            }
        }

        return results;

    }

    private Pose3d getRobotLocation(Transform3d cameraToRobot, Transform3d aprilTagLocation,int aprilTagID){
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(aprilTagID);

        Pose3d robotPose = null;

        if (tagPose.isPresent()){
            Pose3d camPose = tagPose.get().transformBy(aprilTagLocation.inverse());
            robotPose = camPose.transformBy(cameraToRobot);
        }

    return robotPose;
    }

    private double inchToMeter(double inch) {

        return inch * 0.0254;

    }


    public static class Camera {

        private final Transform3d cameraToRobot;
        private final PhotonCamera camera;

        public Camera(Translation3d cameraLocation, Rotation3d cameraDirection, String cameraName) {

            camera = new PhotonCamera(cameraName);
            cameraToRobot = new Transform3d(cameraLocation, cameraDirection).inverse();

        }

        public Transform3d getCameraToRobot() {
            return cameraToRobot;
        }

        public PhotonCamera getCamera() {
            return camera;
        }
    }

}

