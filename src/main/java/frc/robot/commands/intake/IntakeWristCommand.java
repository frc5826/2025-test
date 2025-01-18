package frc.robot.commands.intake;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeWristSubsystem;

import static frc.robot.Constants.Intake.cIntakeDeadband;

public class IntakeWristCommand extends LoggedCommand {

    private IntakeWristSubsystem intakeWristSubsystem;
    private double angle;


    public IntakeWristCommand(IntakeWristSubsystem subsystem, double angleDegrees){

        this.intakeWristSubsystem = subsystem;
        this.angle = angleDegrees;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        super.initialize();

        this.intakeWristSubsystem.setDesiredAngle(angle);

    }

    @Override
    public boolean isFinished() {

        return Math.abs(angle - intakeWristSubsystem.getAngle()) <= cIntakeDeadband;

    }
}


