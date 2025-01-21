package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.localization.Localization;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.util.Optional;

public class SwerveSubsystem extends LoggedSubsystem {

    private final SwerveDrive swerveDrive;

    private double maximumSpeed = Constants.cMaxVelocity;

    private Rotation2d targetAngle = new Rotation2d();

    public SwerveSubsystem(Localization locatization) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                12.8, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(4), 6.12, 1);

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory() + "/swerve")).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        SmartDashboard.putData("/drive/ahrs",(AHRS)swerveDrive.getGyro().getIMU());

        resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false);
    }

    public Rotation2d getIMUYaw() {
        return swerveDrive.getYaw();
    }

    public Optional<Translation3d> getAcc() {
        if(swerveDrive.getAccel().isPresent()) {
            var t = swerveDrive.getAccel().get()
                    .rotateBy(((AHRS)swerveDrive.getGyro().getIMU()).getRotation3d().unaryMinus())
                    .rotateBy(swerveDrive.getGyroRotation3d().unaryMinus());
            return Optional.of(new Translation3d(t.getX(),t.getY(),t.getZ()));
        } else
            return Optional.empty();
    }


    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    public ChassisSpeeds getOdoVel() { return swerveDrive.getFieldVelocity(); }

    public Pose2d getOdoPose()
    {
        return swerveDrive.getPose();
    }

    public void driveRobotOriented(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }

    public void zeroGyro()
    {
        targetAngle = new Rotation2d();
        swerveDrive.zeroGyro();
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void resetOdometry(Pose2d stuff)
    {
        swerveDrive.resetOdometry(stuff);
    }

    private Translation3d toFieldOriented(Translation3d t) {
        return t.rotateBy(new Rotation3d(getIMUYaw().times(-1)));
    }
}
