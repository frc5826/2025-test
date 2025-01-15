package frc.robot.localization;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public record MeasureMetric(RealVector preMeasureX, RealMatrix preMeasureP, RealVector postMeasureX, RealMatrix postMeasureP, RealVector z, RealMatrix R) {
}
