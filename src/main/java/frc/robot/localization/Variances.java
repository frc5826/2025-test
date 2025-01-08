package frc.robot.localization;

public record Variances(double xyPos, double xyVel, double xyAcc,
                        double rPos, double rVel, double rAcc) {
}