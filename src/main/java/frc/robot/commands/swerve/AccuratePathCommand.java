package frc.robot.commands.swerve;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.MathHelper;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

public class AccuratePathCommand extends LoggedCommand {

    private Pose2d goal;

    private SwerveSubsystem swerveSubsystem;

    private boolean finished;

    private final PIDConstants drivePID = new PIDConstants(2.5, 0.1, 0.1);
    private final PID pidX = new PID(drivePID, 1, 0.03, 0.01, () -> swerveSubsystem.getLocalizationPose().getX());
    private final PID pidY = new PID(drivePID, 1, 0.03, 0.01, () -> swerveSubsystem.getLocalizationPose().getY());
    private final PID pidR = new PID(Constants.Swerve.cTurnPID, Math.PI, 0.01, 0.05, () -> swerveSubsystem.getLocalizationPose().getRotation().getRadians());

    public AccuratePathCommand(Pose2d goal, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.goal = goal;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        finished = false;

        if (goal.getTranslation().getDistance(swerveSubsystem.getLocalizationPose().getTranslation()) >= 1) {
            System.out.println("AccuratePathCommand - cannot accept values further than 1m from current pose");
            finished = true;
        }

//        System.out.println("Starting pose: " + swerveSubsystem.getLocalizationPose());
//        System.out.println("Goal pose: " + goal);

        pidX.setGoal(goal.getX());
        pidY.setGoal(goal.getY());
        pidR.setGoal(goal.getRotation().getRadians());
    }

    @Override
    public void execute() {
        pidX.calculate();
        pidY.calculate();
        pidR.calculate();

//        System.out.println("X error: " + pidX.getError());
//        System.out.println("Y error: " + pidY.getError());
//        System.out.println("R error: " + pidR.getError());
//
//        System.out.println("X deadband: " + pidX.getDeadband());
//        System.out.println("Y deadband: " + pidY.getDeadband());
//        System.out.println("R deadband: " + pidR.getDeadband());

        ChassisSpeeds speeds = new ChassisSpeeds(pidX.getOutput(), pidY.getOutput(), pidR.getOutput());
        swerveSubsystem.driveFieldOriented(speeds);

        if (Math.abs(pidX.getError()) <= pidX.getDeadband() && Math.abs(pidY.getError()) <= pidY.getDeadband() && Math.abs(pidR.getError()) <= pidR.getDeadband()) {
            System.out.println("Complete");
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        finished = false;
    }
}
