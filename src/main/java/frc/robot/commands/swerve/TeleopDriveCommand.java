package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.*;

public class TeleopDriveCommand extends LoggedCommand {

    private final SwerveSubsystem swerveSubsystem;

    private final PID turnPID = new PID(4, 0, .4, cMaxAngularVelocity, 0.5, 0.03, this::getTurnDiff);

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        double x = -cXbox.getLeftY();
        double y = -cXbox.getLeftX();

        x = Math.abs(x) > cTeleDriveDeadband ? x : 0;
        y = Math.abs(y) > cTeleDriveDeadband ? y : 0;

        double rx = cXbox.getRightX();
        double ry = cXbox.getRightY();

        double r = Math.hypot(rx, ry);

        rx = r > cTurnDeadband ? rx : 0;
        ry = r > cTurnDeadband ? ry : 0;

        if (Rotation2d.fromRadians(Math.atan2(rx, ry)) != swerveSubsystem.getTargetAngle() && rx != 0 || ry !=0) {
            swerveSubsystem.setTargetAngle( Rotation2d.fromRadians(Math.atan2(-rx, -ry)));
        }

        ChassisSpeeds speeds = new ChassisSpeeds(x * cMaxVelocity * 0.3, y * cMaxVelocity * 0.3, -turnPID.calculate());


        swerveSubsystem.driveFieldOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond)));

        //swerveSubsystem.driveFieldOriented(new ChassisSpeeds(x*maxVelocity, y*maxVelocity, r * maxAngularVelocity));
    }

    private double getTurnDiff() {
        return swerveSubsystem.getTargetAngle().minus(swerveSubsystem.getIMUYaw()).getRadians();
    }
}
