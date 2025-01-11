package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSubsystem;
import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Localization {

    private KalmanFilter kalmanFilter;

    private final Variances measVar;

    private final Timer timer;

    public Localization() {
        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0}, initCovar()));

        this.measVar = new Variances(0, 0, 0, 0, 0, 0);

        timer = new Timer();
    }

    public void move() {
        kalmanFilter.move(timer.get());
        timer.reset();
    }

    public void measure(SwerveSubsystem s) {
        RealVector zOdo = MatrixUtils.createRealVector(new double[]{0, 0, 0,
                s.getOdoVel().vxMetersPerSecond, s.getOdoVel().vyMetersPerSecond, s.getOdoVel().omegaRadiansPerSecond,
                0, 0, 0});

        RealMatrix ROdo = MatrixUtils.createRealMatrix(new double[][]{
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, measVar.xyVel(), 0, 0, 0, 0, 0},
                {0, 0, 0, 0, measVar.xyVel(), 0, 0, 0, 0},
                {0, 0, 0, 0, 0, measVar.rVel(), 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
        });

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

    private static double[][] initCovar() {
        return new double[][]{
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0}, };
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
