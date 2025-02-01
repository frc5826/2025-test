package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveTimeCommand extends Command {

    private double moveSec;

    private SwerveSubsystem swerveSubsystem;
    private Localization localization;

    private final Timer timer;

    private final ChassisSpeeds speeds;

    public MoveTimeCommand(double moveSec, ChassisSpeeds speeds,
                           SwerveSubsystem swerveSubsystem, Localization localization) {

        this.moveSec = moveSec;

        this.speeds = speeds;

        this.swerveSubsystem = swerveSubsystem;
        this.localization = localization;

        this.timer = new Timer();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        timer.restart();
    }

    @Override
    public void execute() {

//        localization.move();
//        localization.measure(swerveSubsystem);
//        localization.updateField();

//        System.out.println("X Acceleration: " + swerveSubsystem.getAccFieldOrient().getX());
//        System.out.println("Y Acceleration: " + swerveSubsystem.getAccFieldOrient().getY())

        if (timer.get() <= moveSec) {
            swerveSubsystem.driveFieldOriented(speeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= moveSec) {
            return true;
        } else {
            return super.isFinished();
        }
    }
}
