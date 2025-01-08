package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveSubsystem;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Localization {

    public void move() {

    }

    public void measure(SwerveSubsystem swerveSubsystem) {

    }

    public void reset() {
        reset(null);
    }

    public void reset(Pose2d pose) {
        if (pose == null) {
            pose = new Pose2d(0, 0, new Rotation2d());
        }
    }
}
