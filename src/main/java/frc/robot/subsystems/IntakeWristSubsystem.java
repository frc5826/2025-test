package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.ArmController;
import frc.robot.math.PID;
import static frc.robot.Constants.Intake.*;

//TODO Possibly a second motor
public class IntakeWristSubsystem extends LoggedSubsystem {

    private SparkMax motor;
    private DutyCycleEncoder encoder;
    private PID intakePID = new PID(0, 0, 0, 0, 0, 0, this::getAngle );
    private ArmController armController = new ArmController(cIntakeV, cIntakeG, cIntakeMaxVelocity, cIntakeMaxAcceleration, cIntakeMinOutput, cIntakeMaxOutput, intakePID);

    public IntakeWristSubsystem(){

        //TODO Device IDs
        motor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(4);

        SmartDashboard.putData("intake/wristPID", intakePID);
        SmartDashboard.putData("intake/wristController", armController);
        SmartDashboard.putData("intake/wristEncoder", encoder);
        SparkMaxConfig config = new SparkMaxConfig();


    }

    //TODO is .02 correct???
    @Override
    public void periodic() {

        double speed = armController.calculate(0.02);
        motor.set(speed);

    }

    public void setDesiredAngle(double angle){

        angle = Math.clamp(angle, cIntakeMinAngle, cIntakeMaxAngle);
        armController.setGoal(getAngle(), angle*(Math.PI/180), motor.getEncoder().getVelocity()*cMotorToRadians);

    }

    public double getAngle(){

        return (encoder.get()+cIntakeOffset)*360;
    }

}



