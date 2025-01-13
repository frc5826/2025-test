package frc.robot.math;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

public class ElevatorController implements NTSendable {

    private double V, G, A;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setPoint;
    private TrapezoidProfile.State goal;
    private PID pid;

    public ElevatorController(double V, double G, double A, double maxVelocity, double maxAcceleration, PID pid){


        this.pid = pid;
        this.V = V;
        this.G = G;
        this.A = A;

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        setPoint = new TrapezoidProfile.State();


    }

    public void setGoal(double startPosition, double endPosition, double startVelocity){

        goal = new TrapezoidProfile.State(endPosition, 0);
        setPoint = new TrapezoidProfile.State(startPosition, startVelocity);


    }

    public double calculate(double deltaTime){

        setPoint = profile.calculate(deltaTime, setPoint, goal);
        pid.setGoal(setPoint.position);
        return setPoint.velocity * V + setPoint.position * G + pid.calculate();

    }

    private double getV() {
        return V;
    }

    private double getG() {
        return G;
    }

    private void setV(double v) {
        V = v;
    }

    private void setG(double g) {
        G = g;
    }

    private double getSetPoint() {
        return setPoint.position;
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {

        builder.setSmartDashboardType("5826-ElevatorController");
        builder.addDoubleProperty("V", this::getV, this::setV );
        builder.addDoubleProperty("G", this::getG, this::setG);
        builder.addDoubleProperty("setPoint", this::getSetPoint, null);
    }
}
