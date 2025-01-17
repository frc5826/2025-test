package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralizerSubsystem extends LoggedSubsystem{

    private SparkMax motor;
    private DigitalInput sensor1, sensor2;

    private CoralizerSubsystem(){
        //TODO Device ID and Channel
        motor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        sensor1 = new DigitalInput(0);
        sensor2 = new DigitalInput(1);
        SmartDashboard.putData("coralizer/coralSensor1", sensor1);
        SmartDashboard.putData("coralizer/coralSensor2", sensor2);

    }

    public void setSpeed(double speed){

        motor.set(speed);

    }

    public boolean hasCoral(){

        return sensor1.get() || sensor2.get();
    }

}
