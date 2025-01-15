package frc.robot.localization;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public record MoveMetric(RealVector preMoveX, RealMatrix preMoveP, RealVector postMoveX, RealMatrix postMoveP, RealMatrix Q) {
}
