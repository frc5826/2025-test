package frc.robot.localization;

import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;

import java.util.Collections;
import java.util.List;


public class Scratch {

    public static void main(String[] args) {

        Localization localization = new Localization();

        localization.move();
        //localization.measure(1, 1, 1);

    }

}
