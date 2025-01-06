package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.localization.Localization;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    public double maximumSpeed = Constants.maxVelocity;

    public SwerveSubsystem(Localization locatization) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                12.8, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(4), 8.14, 1);

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory() + "/swerve")).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        resetOdometry(new Pose2d(4.7, 3.5, new Rotation2d()));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }



    public ChassisSpeeds getVel() { return swerveDrive.getFieldVelocity(); }

    public Pose2d getYAGSLPose()
    {
        return swerveDrive.getPose();
    }

    public void driveRobotOriented(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }

    public Rotation2d getHeading()
    {
        return swerveDrive.getYaw();
    }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

    public void resetOdometry(Pose2d stuff)
    {
        swerveDrive.resetOdometry(stuff);
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    public double getDriveBaseRadius() {
        return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
    }

}
