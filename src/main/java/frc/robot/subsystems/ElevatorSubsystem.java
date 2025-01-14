package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.ElevatorController;
import frc.robot.math.PID;
import org.apache.commons.math3.util.CombinatoricsUtils;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends LoggedSubsystem {

    private PID elevatorPID = new PID(cElevatorP, cElevatorI, cElevatorD, cElevatorMax, cElevatorMin, 0, this::getPos);
    private ElevatorController controller = new ElevatorController(cElevatorV, cElevatorG, cElevatorMaxVelocity, cElevatorMaxAcceleration, cElevatorMin, cElevatorMax, elevatorPID );
    //private  motor, motorFollower;

    public ElevatorSubsystem(){
        SmartDashboard.putData("elevator/PID", elevatorPID);
        SmartDashboard.putData("elevator/Controller", controller);
        //motorFollower.follow
    }

    @Override
    public void periodic() {

        double speed = controller.calculate(0.02);
       // motor.set(speed);

    }

    //TODO Get actual positions
    public double getPos(){
        return 0.0;
    }

}
