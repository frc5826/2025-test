package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MathHelper {

    public static Rotation2d getAngleAtoB(Pose2d A, Pose2d B) {
        return B.getTranslation().minus(A.getTranslation()).getAngle();
    }

    public static double fixSpin(double angle) {
        if(angle < -Math.PI){
            angle += 2*Math.PI;
        } else if (angle > Math.PI) {
            angle -= 2*Math.PI;
        }

        return angle;
    }

}
