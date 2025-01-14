package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;

public class TeleopDriveCommand extends LoggedCommand {

    private final SwerveSubsystem swerveSubsystem;

    private final PID turnPID = new PID(4, 0, .4, maxAngularVelocity, 0.5, 0.03, this::getTurnDiff);

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        double x = xbox.getLeftY();
        double y = xbox.getLeftX();

        x = Math.abs(x) > teleDriveDeadband ? x : 0;
        y = Math.abs(y) > teleDriveDeadband ? y : 0;

        double rx = xbox.getRightX();
        double ry = xbox.getRightY();

        double r = Math.hypot(rx, ry);

        rx = r > turnDeadband ? rx : 0;
        ry = r > turnDeadband ? ry : 0;

        if (Rotation2d.fromRadians(Math.atan2(rx, ry)) != targetAngle && rx != 0 || ry !=0) {
            targetAngle = Rotation2d.fromRadians(Math.atan2(rx, ry));
        }

        ChassisSpeeds speeds = new ChassisSpeeds(x * swerveSubsystem.maximumSpeed * 0.3, y * swerveSubsystem.maximumSpeed * 0.3, turnPID.calculate());

        //TODO
        swerveSubsystem.driveFieldOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond)));

        //swerveSubsystem.driveFieldOriented(new ChassisSpeeds(x*maxVelocity, y*maxVelocity, r * maxAngularVelocity));
//        if (xbox.getAButtonPressed()) {
//            swerveSubsystem.zeroGyro();
//        }
    }

    private double getTurnDiff() {
        return targetAngle.minus(swerveSubsystem.getIMUYaw()).minus(Rotation2d.fromDegrees(180)).getRadians();
    }
}
