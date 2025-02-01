package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;

public class PathToCommand extends LoggedCommand {

    private SwerveSubsystem swerveSubsystem;

    private Pose2d goal;

    private PathConstraints constraints;

    private double endVel;

    private Command pathCommand;

    private boolean finished;

    public PathToCommand(Pose2d pose, double endVel, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        goal = pose;
        this.constraints = constraints;
        this.endVel = endVel;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        Pose2d start = swerveSubsystem.getLocalizationPose();
        start = new Pose2d(start.getTranslation(), MathHelper.getAngleAtoB(start, goal));

        Pose2d end = new Pose2d(goal.getTranslation(), MathHelper.getAngleAtoB(start, goal));

        SmartDashboard.putNumber("test/start", start.getRotation().getDegrees());
        SmartDashboard.putNumber("test/goal", end.getRotation().getDegrees());

        List<Waypoint> points = PathPlannerPath.waypointsFromPoses(start, end);

        PathPlannerPath path = new PathPlannerPath(
                points,
                constraints,
                null,
                new GoalEndState(endVel, goal.getRotation()));

        swerveSubsystem.setTargetAngle(goal.getRotation());

        pathCommand = AutoBuilder.followPath(path);


        try {
            pathCommand.initialize();
        } catch(Exception e) {
            finished = true;
            System.err.println("Start: " + start + "\nEnd: " + end);
        }
    }

    @Override
    public void execute() {
        super.execute();

        pathCommand.execute();

        finished = pathCommand.isFinished();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        finished = false;

        pathCommand.end(interrupted);
    }

}
