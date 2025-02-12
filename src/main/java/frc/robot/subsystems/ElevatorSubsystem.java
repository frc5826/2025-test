package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.ElevatorController;
import frc.robot.math.PID;

import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends LoggedSubsystem {

    private PID elevatorPID = new PID(cElevatorP, cElevatorI, cElevatorD, cElevatorMaxOutput, cElevatorMinOutput, 0, this::getPos);
    private ElevatorController controller = new ElevatorController(cElevatorV, cElevatorG, cElevatorMaxVelocity, cElevatorMaxAcceleration, cElevatorMinOutput, cElevatorMaxOutput, elevatorPID );
    private SparkMax motor, motorFollower;
    private Encoder encoder;

    public ElevatorSubsystem(){

        motor = new SparkMax(cElevatorMotor1ID, SparkLowLevel.MotorType.kBrushless);
        motorFollower = new SparkMax(cElevatorMotor2ID, SparkLowLevel.MotorType.kBrushless);
        encoder = new Encoder(cElevatorEncoder1IDA, cElevatorEncoder1IDB);
        SmartDashboard.putData("elevator/PID", elevatorPID);
        SmartDashboard.putData("elevator/Controller", controller);
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(motor);
        motor.setInverted(true);
        motorFollower.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }
        //TODO is .02 correct???
    @Override
    public void periodic() {

        double speed = controller.calculate(0.02);
        motor.set(speed);

    }

    public void setDesiredPosition(double position) {

        position = Math.clamp(position, cElevatorHeightMin, cElevatorHeightMax);
        controller.setGoal(getPos(), position, motor.getEncoder().getVelocity()* cElevatorRPMtoMPS);

    }


    public double getPos(){

        return encoder.get()/ cElevatorClicksPerMeter;

    }

}
