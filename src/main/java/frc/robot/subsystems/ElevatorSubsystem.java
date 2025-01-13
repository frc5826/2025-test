package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.ElevatorController;
import frc.robot.math.PID;

public class ElevatorSubsystem extends LoggedSubsystem {

    private PID elevatorPID = new PID(0, 0, 0, 0, 0, 0, this::getPos);
    private ElevatorController controller = new ElevatorController(0, 0, 0, 0, 0, elevatorPID );

    public ElevatorSubsystem(){
        SmartDashboard.putData("elevator/PID", elevatorPID);
        SmartDashboard.putData("elevator/Controller", controller);
    }

    //TODO Get actual positions
    public double getPos(){
        return 0.0;
    }

}
