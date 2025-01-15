package frc.robot.math;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

public class ElevatorController implements NTSendable {

    private double V, G;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setPoint;
    private TrapezoidProfile.State goal;
    private PID pid;
    private double min, max;
    private double output;


    public ElevatorController(double V, double G, double maxVelocity, double maxAcceleration, double min, double max, PID pid){


        this.pid = pid;
        this.V = V;
        this.G = G;
        this.min = min;
        this.max = max;

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        setPoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();


    }

    public void setGoal(double startPosition, double endPosition, double startVelocity){

        goal = new TrapezoidProfile.State(endPosition, 0);
        setPoint = new TrapezoidProfile.State(startPosition, startVelocity);


    }

    public double calculate(double deltaTime){


        setPoint = profile.calculate(deltaTime, setPoint, goal);
        pid.setGoal(setPoint.position);
        output = setPoint.velocity * V + G + pid.calculate();
        return Math.clamp(output, min, max);

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

    private double getOutput() {
        return output;
    }

    private double getGoal() {
        return goal.position;
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {

        builder.setSmartDashboardType("5826-ElevatorController");
        builder.addDoubleProperty("V", this::getV, this::setV );
        builder.addDoubleProperty("G", this::getG, this::setG);
        builder.addDoubleProperty("setPoint", this::getSetPoint, null);
        builder.addDoubleProperty("Output", this::getOutput, null);
        builder.addDoubleProperty("Goal", this::getGoal, null);
    }
}
