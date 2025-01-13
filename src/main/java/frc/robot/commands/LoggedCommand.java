package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggedSubsystem;

public class LoggedCommand extends Command {
    @Override
    public void initialize() {
        for(Subsystem s : getRequirements()) {
            if(s instanceof LoggedSubsystem) {
                ((LoggedSubsystem) s).updateCommand(this);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for(Subsystem s : getRequirements()) {
            if(s instanceof LoggedSubsystem) {
                ((LoggedSubsystem) s).updateCommand(null);
            }
        }
    }
}
