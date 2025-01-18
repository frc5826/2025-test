package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.ArmController;
import frc.robot.math.PID;
import static frc.robot.Constants.Coralizer.*;

public class CoralizerWristSubsystem extends LoggedSubsystem{

    private SparkMax motor;
    private DutyCycleEncoder encoder;
    private PID coralizerPID = new PID(0, 0, 0, 0, 0, 0, this::getAngle );
    private ArmController armController = new ArmController(cWristV, cWristG, cWristMaxVelocity, cWristMaxAcceleration, cWristMinOutput, cWristMaxOutput, coralizerPID);

    public CoralizerWristSubsystem(){

        //TODO Device IDs
        motor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(3);
        SmartDashboard.putData("coralizer/wristPID", coralizerPID);
        SmartDashboard.putData("coralizer/wristController", armController);
        SmartDashboard.putData("coralizer/wristEncoder", encoder);
        SparkMaxConfig config = new SparkMaxConfig();


    }

    //TODO is .02 correct???
    @Override
    public void periodic() {

        double speed = armController.calculate(0.02);
        motor.set(speed);

    }

    public void setDesiredAngle(double angle){

        angle = Math.clamp(angle, cWristMinAngle, cWristMaxAngle);
        armController.setGoal(getAngle(), angle*(Math.PI/180), motor.getEncoder().getVelocity()*cMotorToRadians);

    }

    public double getAngle(){

        return (encoder.get()+cWristOffset)*360;
    }

}
