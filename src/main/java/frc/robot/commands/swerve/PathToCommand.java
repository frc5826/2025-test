package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LoggedCommand;
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

        List<Waypoint> points = PathPlannerPath.waypointsFromPoses(
                swerveSubsystem.getLocalizationPose(),
                goal
        );

        PathPlannerPath path = new PathPlannerPath(
                points,
                constraints,
                null,
                new GoalEndState(endVel, goal.getRotation()));

        pathCommand = AutoBuilder.followPath(path);

        pathCommand.initialize();
    }

    @Override
    public void execute() {
        super.execute();

        pathCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        pathCommand.end(interrupted);
    }

}
