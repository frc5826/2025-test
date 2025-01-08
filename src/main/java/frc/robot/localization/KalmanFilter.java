package frc.robot.localization;

import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

class KalmanFilter {

    private RealVector x; //Initial means
    private RealMatrix P;//Initial covariances

    private RealMatrix Q; //Movement covariance of the process noise

    //Used if we don't supply a B and u vector
    private RealMatrix defaultB; //Control inputs transition
    private RealVector defaultU; //Control inputs

    private Variances var;

    public KalmanFilter(MultivariateNormalDistribution initial) {
        this.x = MatrixUtils.createRealVector(initial.getMeans());
        this.P = initial.getCovariances();

        this.var = new Variances(0, 0, 0, 0, 0, 0);

        this.Q = MatrixUtils.createRealMatrix(new double[][]{
                //X pos variance
                {var.xyPos(), 0, 0, 0, 0, 0, 0, 0, 0},
                {0, var.xyPos(), 0, 0, 0, 0, 0, 0, 0},
                {0, 0, var.rPos(), 0, 0, 0, 0, 0, 0},
                {0, 0, 0, var.xyVel(), 0, 0, 0, 0, 0},
                {0, 0, 0, 0, var.xyVel(), 0, 0, 0, 0},
                {0, 0, 0, 0, 0, var.rVel(), 0, 0, 0},
                {0, 0, 0, 0, 0, 0, var.xyAcc(), 0, 0},
                {0, 0, 0, 0, 0, 0, 0, var.xyAcc(), 0},
                {0, 0, 0, 0, 0, 0, 0, 0, var.rAcc()},
        });

        this.defaultB = MatrixUtils.createRealIdentityMatrix(this.x.getDimension());
        this.defaultU = MatrixUtils.createRealVector(new double[this.x.getDimension()]);
    }

    private void move(double deltaTime, RealMatrix B, RealVector u, RealMatrix Q) {
        RealMatrix F = getF(deltaTime);

        x = F.operate(x).add(B.operate(u));
        P = F.multiply(P).multiply(F.transpose()).add(Q);
    }

    public void move(double deltaTime, RealMatrix Q){
        this.move(deltaTime, defaultB, defaultU, Q);
    }

    public void measure(RealMatrix H, RealMatrix R, RealVector z) {
        RealMatrix S = H.multiply(P).multiply(H.transpose()).add(R);
        RealMatrix K = P.multiply(H.transpose()).multiply(MatrixUtils.inverse(S));
        RealVector y = z.subtract((H.operate(x)));
        x = x.add(K.operate(y));

        RealMatrix KH = K.multiply(H);
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(KH.getRowDimension());
        P = I.subtract(KH).multiply(P);
    }

    //State transition matrix
    private RealMatrix getF(double deltaT){
        return MatrixUtils.createRealMatrix(new double[][]{
                {1, 0, 0, deltaT, 0, 0, Math.pow(deltaT, 2) / 2, 0, 0}, // xp
                {0, 1, 0, 0, deltaT, 0, 0, Math.pow(deltaT, 2) / 2, 0}, // yp
                {0, 0, 1, 0, 0, deltaT, 0, 0, Math.pow(deltaT, 2) / 2}, // rp
                {0, 0, 0, 1, 0, 0, deltaT, 0, 0}, // xv
                {0, 0, 0, 0, 1, 0, 0, deltaT, 0}, // yv
                {0, 0, 0, 0, 0, 1, 0, 0, deltaT}, // rv
                {0, 0, 0, 0, 0, 0, 1, 0, 0}, // xa
                {0, 0, 0, 0, 0, 0, 0, 1, 0}, // ya
                {0, 0, 0, 0, 0, 0, 0, 0, 1}}); // ra
    }

    public MultivariateNormalDistribution getEstimate() {
        return new MultivariateNormalDistribution(x.toArray(), P.getData());
    }
}
