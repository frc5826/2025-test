package frc.robot.commands.elevator;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerWristSubsystem;

import static frc.robot.Constants.Coralizer.*;
import static frc.robot.Constants.Elevator.cElevatorDeadband;

public class CoralizerWristCommand extends LoggedCommand {

    private CoralizerWristSubsystem coralizerWristSubsystem;
    private double angle;


    public CoralizerWristCommand(CoralizerWristSubsystem subsystem, double angleDegrees){

        this.coralizerWristSubsystem = subsystem;
        this.angle = angleDegrees;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        super.initialize();

        this.coralizerWristSubsystem.setDesiredAngle(angle);

    }

    @Override
    public boolean isFinished() {

        return Math.abs(angle - coralizerWristSubsystem.getAngle()) <= cWristDeadband;

    }
}
