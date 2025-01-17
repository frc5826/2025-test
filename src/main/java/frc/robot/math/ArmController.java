package frc.robot.math;

public class ArmController extends ElevatorController {


    public ArmController(double V, double G, double maxVelocity, double maxAcceleration, double min, double max, PID pid) {
        super(V, G, maxVelocity, maxAcceleration, min, max, pid);
    }

    @Override
    public double calculate(double deltaTime) {

        setPoint = profile.calculate(deltaTime, setPoint, goal);
        pid.setGoal(setPoint.position);
        output = setPoint.velocity * V + G * Math.cos(setPoint.position) + pid.calculate();
        return Math.clamp(output, min, max);
    }
}
