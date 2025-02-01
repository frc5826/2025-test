package frc.robot.localization;

import com.google.errorprone.annotations.Var;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private FilterBuffer buffer;

    private Field2d field;

    public Localization() {
        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0}, initCovar()));

        this.measVar = new Variances(2, 0.4, 2, 2, 0.4, 2);

        cameras = new CameraSystem();

        timer = new Timer();

        buffer = new FilterBuffer(25, 0.02);

        field = new Field2d();
        setupField();
    }

    public void move(int counter) {
        double dt = timer.get();
        timer.restart();
//        if(counter % 10 == 0) {
//            System.out.println("Localization dt @ " + counter + ": " + dt);
//        }
        kalmanFilter.move(dt);
    }

    public void measure(SwerveSubsystem s) {
        RealVector zOdo = kalmanFilter.getX().copy();
        RealMatrix ROdo = kalmanFilter.getP().copy();

        Optional<Translation3d> accelOptional = s.getAcc();
        if (accelOptional.isPresent()) {
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

        zOdo.setEntry(2, s.getRotationCorrected().getRadians());

        SmartDashboard.putNumber("corrected gyro yaw", s.getRotationCorrected().getDegrees());

        ROdo.setEntry(2, 2, measVar.rPos());

        for (var p : cameras.getCameraMeasurements()) {
            try {
                if (p.getSecond() > 10) {
                    addVisionMeasurement(p.getFirst(), p.getSecond());
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        //for all you readings...

        // copy z and r like above
        // enter pos in z
        // variance (r) comes from confidence of the april tag

        buffer.addSnapshot(new FilterSnapshot(NetworkTablesJNI.now() / 1000000.0, kalmanFilter.getX(), zOdo, ROdo));
        kalmanFilter.measure(ROdo, zOdo);
    }

    private void addVisionMeasurement(Pose3d p, double time) {
        var snapshotPair = buffer.getSnapshot(time);

        if (snapshotPair.isPresent()) {
            FilterSnapshot snapshot = snapshotPair.get().getFirst();
            int index = snapshotPair.get().getSecond();

            System.out.println("index: " + index);

            RealVector z = snapshot.z();
            RealMatrix R = snapshot.R();

            z.setEntry(0, p.getX());
            z.setEntry(1, p.getY());

            R.setEntry(0, 0, measVar.xyPos());
            R.setEntry(1, 1, measVar.xyPos());

            z.setEntry(2, p.getRotation().getZ());

            R.setEntry(2, 2, measVar.rPos());

            kalmanFilter.setX(snapshot.x());
            kalmanFilter.setP(snapshot.R());

            kalmanFilter.measure(R, z);

//            if (index > 0) {
//                FilterSnapshot[] replay = buffer.getReplay(index);
//
//                kalmanFilter.move(replay[0].time() - snapshot.time());
//
//                for (int i = 0; i + 1 < replay.length; i++) {
//                    buffer.addSnapshot(
//                            new FilterSnapshot(replay[i].time(),
//                                    kalmanFilter.getX(),
//                                    replay[i].z(),
//                                    replay[i].R()),
//                            replay.length - i);
//                    kalmanFilter.measure(replay[i].R(), replay[i].z());
//                    kalmanFilter.move(replay[i + 1].time() - replay[i].time());
//                }
//            }
        }


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
                .withPosition(2, 0)
                .withSize(5, 3);

        ShuffleboardLayout position = tab.getLayout("Robot position", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 2);

        position.addDouble("Robot X", () -> getPose().getX());
        position.addDouble("Robot Y", () -> getPose().getY());
        position.addDouble("Robot rotation", () -> getPose().getRotation().getDegrees());
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
