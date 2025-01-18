package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoggedSubsystem;

public class IntakeCommandPush extends LoggedCommand {


    private IntakeSubsystem subsystem;
    private Timer timer;

    public IntakeCommandPush(IntakeSubsystem subsystem) {

        timer = new Timer();
        this.subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        super.initialize();
        //TODO
        this.subsystem.setSpeed(-1);
        timer.restart();

    }

    @Override
    public boolean isFinished() {

        return !this.subsystem.hasCoral() && timer.get() > 1;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        this.subsystem.setSpeed(0);
        timer.stop();
    }
}