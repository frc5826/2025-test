package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.CameraSystem;
import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Optional;

public class Localization {

    private KalmanFilter kalmanFilter;

    private final Variances measVar;

    private final CameraSystem cameras;

    private final Timer timer;

    private Field2d field;

    public Localization() {
        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0}, initCovar()));

        this.measVar = new Variances(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        cameras = new CameraSystem();

        timer = new Timer();

        field = new Field2d();
        setupField();
    }

    public void move() {
        kalmanFilter.move(timer.get());
        timer.restart();
    }

    public void measure(SwerveSubsystem s) {
        RealVector zOdo = kalmanFilter.getX().copy();
        RealMatrix ROdo = kalmanFilter.getP().copy();

        Optional<Translation3d> accelOptional = s.getAcc();
        if(accelOptional.isPresent()){
            zOdo.setEntry(6, accelOptional.get().getX());
            zOdo.setEntry(7, accelOptional.get().getY());

            ROdo.setEntry(6, 6, measVar.xyAcc());
            ROdo.setEntry(7, 7, measVar.xyAcc());
        }

//        Velocity = wheels
        ChassisSpeeds velocity = s.getOdoVel();
        zOdo.setEntry(3, velocity.vxMetersPerSecond);
        zOdo.setEntry(4, velocity.vyMetersPerSecond);
        zOdo.setEntry(5, velocity.omegaRadiansPerSecond);

        ROdo.setEntry(3, 3, measVar.xyVel());
        ROdo.setEntry(4, 4, measVar.xyVel());
        ROdo.setEntry(5, 5, measVar.rVel());

        //Position

        zOdo.setEntry(2, s.getOdoPose().getRotation().getRadians());

        ROdo.setEntry(2, 2, measVar.rPos());

        for(Pose3d p: cameras.getCameraMeasurements()) {
            double ambiguity = cameras.getPoseAmbiguity();

            zOdo.setEntry(0, p.getX());
            zOdo.setEntry(1, p.getY());

            ROdo.setEntry(0, 0, ambiguity);
            ROdo.setEntry(1, 1, ambiguity);

            zOdo.setEntry(2, p.getRotation().getZ());

            ROdo.setEntry(2, 2, ambiguity);
        }

        //for all you readings...

            // copy z and r like above
            // enter pos in z
            // variance (r) comes from confidence of the april tag

        kalmanFilter.measure(ROdo, zOdo);

    }

    public Pose2d getPose() {
        RealVector m = kalmanFilter.getX();

        Pose2d pose = new Pose2d(
                m.getEntry(0),
                m.getEntry(1),
                new Rotation2d(m.getEntry(2)));

        return pose;
    }

    public void updateField() {
        field.setRobotPose(getPose());
    }

    private void setupField() {
        ShuffleboardTab tab = Shuffleboard.getTab("field");

        field = new Field2d();
        tab.add(field)
                .withPosition(2,0)
                .withSize(5,3);

        ShuffleboardLayout position = tab.getLayout("Robot position", BuiltInLayouts.kList)
                .withPosition(0,0)
                .withSize(2,2);

        position.addDouble("Robot X", ()-> getPose().getX());
        position.addDouble("Robot Y", ()-> getPose().getY());
        position.addDouble("Robot rotation", ()-> getPose().getRotation().getDegrees());
    }

    private static double[][] initCovar() {
        return new double[][]{
                {0.1, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0.1, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0.1, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0.1, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0.1, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0.1, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0.1, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0.1, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0.1}};
    }

    public void reset() {
        reset(null);
    }

    public void reset(Pose2d pose) {
        if (pose == null) {
            pose = new Pose2d(0, 0, new Rotation2d());
        }

        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(
                new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians(),
                        0, 0, 0, 0, 0, 0}, initCovar()));

    }
}
