package frc.robot.vision;

import com.revrobotics.spark.config.LimitSwitchConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private DoubleLogEntry xLog, yLog, rotationLog, ambiguityLog;


    public CameraSystem() {

        try {
            fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/field/TestField2025.json");
            fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            fieldLayout = null;
            System.err.println("April Tag Field Layout Failed to Load");
            e.printStackTrace();
        }
        cameras = List.of(
                new Camera(new Translation3d(inchToMeter(7.75), inchToMeter(-4), inchToMeter(8.25)), new Rotation3d(Math.PI, -Math.PI / 4, 0), "alpha")
                //new Camera(new Translation3d(inchToMeter(-9), inchToMeter(-7), inchToMeter(8.25)), new Rotation3d(0, -Math.PI / 6, Math.PI), "beta")
        );

        DataLog log = DataLogManager.getLog();
        xLog = new DoubleLogEntry(log, "/robot/vision/position/x");
        yLog = new DoubleLogEntry(log, "/robot/vision/position/y");
        rotationLog = new DoubleLogEntry(log, "/robot/vision/position/rotation");
        ambiguityLog = new DoubleLogEntry(log, "/robot/vision/ambiguity");

    }

    public List<Pair<Pose3d, Double>> getCameraMeasurements() {

        LinkedList<Pair<Pose3d, Double>> results = new LinkedList<>();

        for (Camera camera : cameras) {

            for (PhotonPipelineResult result : camera.getCamera().getAllUnreadResults()) {

                if (result.hasTargets()) {

                    for (PhotonTrackedTarget target : result.getTargets()) {
                        if (target.getFiducialId() > -1 && target.getPoseAmbiguity() <= POSE_CUTOFF && target.getPoseAmbiguity() != -1) {

                            Pose3d robotPose = getRobotLocation(camera.getCameraToRobot(), target.getBestCameraToTarget(), target.getFiducialId());

                            xLog.append(robotPose.getX());
                            yLog.append(robotPose.getY());
                            rotationLog.append(robotPose.getRotation().getZ());
                            ambiguityLog.append(target.getPoseAmbiguity());

                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/x", robotPose.getX());
                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/y", robotPose.getY());
                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/yaw", robotPose.getRotation().getZ());
                            SmartDashboard.putNumber("Ambiguity", target.getPoseAmbiguity());

                            results.add(Pair.of(robotPose, result.getTimestampSeconds()));
                        }

                    }
                }

            }
        }

        return results;

    }

    public double getPoseAmbiguity() {
        return ambiguityLog.getLastValue();
    }

    private Pose3d getRobotLocation(Transform3d cameraToRobot, Transform3d aprilTagLocation,int aprilTagID){
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(aprilTagID);

        Pose3d robotPose = null;



        if (tagPose.isPresent()){
            Pose3d camPose = tagPose.get().transformBy(aprilTagLocation.inverse());

            SmartDashboard.putNumber("pre transform x", camPose.getX());
            SmartDashboard.putNumber("pre transform Y", camPose.getY());
            SmartDashboard.putNumber("pre transform yaw", Math.toDegrees(camPose.getRotation().getZ()));

            robotPose = camPose.transformBy(cameraToRobot);

            SmartDashboard.putNumber("post transform x", robotPose.getX());
            SmartDashboard.putNumber("post transform Y", robotPose.getY());
            SmartDashboard.putNumber("post transform yaw", Math.toDegrees(robotPose.getRotation().getZ()));
        }

    return robotPose;
    }

    private double inchToMeter(double inch) { return inch * 0.0254; }


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

        public String getName() { return camera.getName(); }
    }

}

