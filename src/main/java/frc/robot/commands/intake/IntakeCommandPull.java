package frc.robot.commands.intake;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandPull extends LoggedCommand {

    private IntakeSubsystem subsystem;

    public IntakeCommandPull(IntakeSubsystem subsystem){

        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        subsystem.setSpeed(1);
    }

    @Override
    public boolean isFinished() {

        return subsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        subsystem.setSpeed(0);
    }
}
