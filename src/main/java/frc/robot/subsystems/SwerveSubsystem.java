package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.localization.Localization;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import static frc.robot.Constants.Swerve.*;

import java.io.File;
import java.util.Optional;

public class SwerveSubsystem extends LoggedSubsystem {

    private final SwerveDrive swerveDrive;

    private double maximumSpeed = cMaxVelocity;

    private Rotation2d targetAngle = new Rotation2d();

    private Localization localization;

    //test
    private double testXpos;
    private final Timer timer;

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

        this.localization = locatization;

        setupPathPlanner();

        timer = new Timer();
        timer.start();
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

    public void setupPathPlanner()
    {

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            config = null;
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                localization::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        cDrivePID, // Translation PID constants
                        cTurnPID // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void driveRotations(double rotations) {
        swerveDrive.getModules()[0].getDriveMotor().setPosition(rotations);
    }

    public void prePeriodic(int counter) {
        double dt = timer.get();
//        if(counter % 10 == 0) {
//            System.out.println("Swerve dt @ " + counter + ": " + dt);
//        }
        testXpos += getOdoVel().vxMetersPerSecond * dt;
        timer.restart();

        SmartDashboard.putNumber("Drive motor pos", swerveDrive.getModules()[0].getDriveMotor().getPosition() / (4 * Math.PI * 0.0254));
        SmartDashboard.putNumber("Odo pose x", getOdoPose().getX());
        SmartDashboard.putNumber("Test odo pose x", testXpos);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void resetTestOdoPose(){
        testXpos = 0;
    }

    public Rotation2d getRotationCorrected() {
        return getIMUYaw().getRadians() < 0 ? getIMUYaw().plus(new Rotation2d(2 * Math.PI)) : getIMUYaw();
    }

    public Pose2d getLocalizationPose() { return localization.getPose(); }

    public Translation3d getAccFieldOrient() { return toFieldOriented(swerveDrive.getAccel().get()); }

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

    public void offsetGyro(double offsetRad) {
        swerveDrive.getGyro().setOffset(new Rotation3d(0, 0, offsetRad));
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
