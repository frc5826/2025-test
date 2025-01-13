package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggedSubsystem extends SubsystemBase {
    private final StringPublisher currentCommand = NetworkTableInstance.getDefault().
            getStringTopic(getName()).publish();
    public void updateCommand(Command c) {
        if(c == null) {
            currentCommand.set("");
        } else {
            currentCommand.set(c.getName());
        }
    }
}
