package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends LoggedSubsystem{

    private SparkMax motor, motorFollower;
    private DigitalInput sensor;

    private IntakeSubsystem(){
        //TODO Device ID and Channel
        motor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(3);
        SmartDashboard.putData("intake/intakeSensor", sensor);
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(motor);
        config.inverted(true);
        motorFollower.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public void setSpeed(double speed){

        motor.set(speed);

    }

    public boolean hasCoral(){

        return sensor.get();
    }

}
