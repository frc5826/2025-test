package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.SwerveSubsystem;
import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Localization {

    private KalmanFilter kalmanFilter;

    private final Variances measVar;

    private final Timer timer;

    private Field2d field;

    public Localization() {
        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0}, initCovar()));

        this.measVar = new Variances(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        timer = new Timer();

        field = new Field2d();
        setupField();
    }

    public void move() {
        kalmanFilter.move(timer.get());
        timer.restart();
    }

    public void measure(SwerveSubsystem s) {
        RealVector zOdo = MatrixUtils.createRealVector(new double[]{
                kalmanFilter.getX().getEntry(0), kalmanFilter.getX().getEntry(1), s.getIMUYaw().getRadians(),
                s.getOdoVel().vxMetersPerSecond, s.getOdoVel().vyMetersPerSecond, s.getOdoVel().omegaRadiansPerSecond,
                s.getAcc().get().getX(), s.getAcc().get().getY(), kalmanFilter.getX().getEntry(8)});

        RealMatrix ROdo = MatrixUtils.createRealMatrix(new double[][]{
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, measVar.rPos(), 0, 0, 0, 0, 0, 0},
                {0, 0, 0, measVar.xyVel(), 0, 0, 0, 0, 0},
                {0, 0, 0, 0, measVar.xyVel(), 0, 0, 0, 0},
                {0, 0, 0, 0, 0, measVar.rVel(), 0, 0, 0},
                {0, 0, 0, 0, 0, 0, measVar.xyAcc(), 0, 0},
                {0, 0, 0, 0, 0, 0, 0, measVar.xyAcc(), 0},
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
