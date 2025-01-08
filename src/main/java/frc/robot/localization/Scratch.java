package frc.robot.localization;

import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;


public class Scratch {

    public static void main(String[] args) {
        new frc.robot.localization.KalmanFilter(new MultivariateNormalDistribution(new double[9], MatrixUtils.createRealIdentityMatrix(9).getData()));
    }

}
