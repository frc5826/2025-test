package frc.robot.localization;

import com.google.gson.Gson;
import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

class KalmanFilter {

    private RealVector x; //Initial means
    private RealMatrix P;//Initial covariances

    private RealMatrix Q; //Movement covariance of the process noise

    private RealMatrix H; //Maps state to measurement

    //Used if we don't supply a B and u vector
    private RealMatrix defaultB; //Control inputs transition
    private RealVector defaultU; //Control inputs

    private final Variances moveVar;

    private MoveMetric mm;

    private Gson gson = new Gson();

    public KalmanFilter(MultivariateNormalDistribution initial) {
        this.x = MatrixUtils.createRealVector(initial.getMeans());
        this.P = initial.getCovariances();

        this.moveVar = new Variances(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        this.Q = MatrixUtils.createRealMatrix(new double[][]{
                //X pos variance
                {moveVar.xyPos(), 0, 0, 0, 0, 0, 0, 0, 0},
                {0, moveVar.xyPos(), 0, 0, 0, 0, 0, 0, 0},
                {0, 0, moveVar.rPos(), 0, 0, 0, 0, 0, 0},
                {0, 0, 0, moveVar.xyVel(), 0, 0, 0, 0, 0},
                {0, 0, 0, 0, moveVar.xyVel(), 0, 0, 0, 0},
                {0, 0, 0, 0, 0, moveVar.rVel(), 0, 0, 0},
                {0, 0, 0, 0, 0, 0, moveVar.xyAcc(), 0, 0},
                {0, 0, 0, 0, 0, 0, 0, moveVar.xyAcc(), 0},
                {0, 0, 0, 0, 0, 0, 0, 0, moveVar.rAcc()},
        });

        this.H = MatrixUtils.createRealIdentityMatrix(9);

        this.defaultB = MatrixUtils.createRealIdentityMatrix(this.x.getDimension());
        this.defaultU = MatrixUtils.createRealVector(new double[this.x.getDimension()]);

        try {
            Files.createDirectories(Paths.get("/tmp/kalman"));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void move(double deltaTime, RealMatrix B, RealVector u, RealMatrix Q) {
        RealVector preMoveX = x.copy();
        RealMatrix preMoveP = P.copy();

        RealMatrix F = getF(deltaTime);
        x = F.operate(x).add(B.operate(u));
        P = F.multiply(P).multiply(F.transpose()).add(Q);

        mm = new MoveMetric(preMoveX, preMoveP, x.copy(), P.copy(), Q.copy());
    }

    public void move(double deltaTime){
        this.move(deltaTime, defaultB, defaultU, this.Q);
    }

    public void measure(RealMatrix R, RealVector z) {
        RealVector preMeasureX = x.copy();
        RealMatrix preMeasureP = P.copy();

        RealMatrix S = H.multiply(P).multiply(H.transpose()).add(R);
        RealMatrix K = P.multiply(H.transpose()).multiply(MatrixUtils.inverse(S));
        RealVector y = z.subtract((H.operate(x)));
        x = x.add(K.operate(y));

        RealMatrix KH = K.multiply(H);
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(KH.getRowDimension());
        P = I.subtract(KH).multiply(P);

        MeasureMetric mem = new MeasureMetric(preMeasureX, preMeasureP, x.copy(), P.copy(), z.copy(), R.copy());

        Metric m = new Metric(System.currentTimeMillis(), mm, mem, new TruthMetric(List.of()));
//        try {
//            FileWriter fw = new FileWriter("/tmp/kalman/" + m.time() + ".metric");
//            gson.toJson(m, fw);
//            fw.close();
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        } finally {
//            mm = null;
//        }
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

    public RealVector getX() {
        //System.out.println("kalmanfilter x: " + x);
        return x;
    }

    public RealMatrix getP() {
        return P;
    }
}
