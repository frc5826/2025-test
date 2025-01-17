package frc.robot.commands.elevator;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerIntakeCommand extends LoggedCommand {

    private CoralizerSubsystem subsystem;

    public CoralizerIntakeCommand(CoralizerSubsystem subsystem){

        this.subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        super.initialize();
        //TODO
        this.subsystem.setSpeed(1);

    }

    @Override
    public boolean isFinished() {

        return this.subsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        this.subsystem.setSpeed(0);
    }
}
