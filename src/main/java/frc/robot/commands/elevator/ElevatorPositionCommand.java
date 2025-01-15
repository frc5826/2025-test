package frc.robot.commands.elevator;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.Elevator.*;

public class ElevatorPositionCommand extends LoggedCommand {

    private ElevatorSubsystem elevatorSubsystem;
    private double position;


    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position){

        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;

        addRequirements(elevatorSubsystem);

    }

    @Override
    public void initialize() {
        super.initialize();

        elevatorSubsystem.setDesiredPosition(position);

    }

    @Override
    public boolean isFinished() {

        return Math.abs(position - elevatorSubsystem.getPos()) <= cElevatorDeadband;

    }
}
