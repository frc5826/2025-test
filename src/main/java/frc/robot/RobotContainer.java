// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.AccuratePathCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.commands.swerve.PathToCommand;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.commands.TestVarianceCommand;
import frc.robot.localization.Localization;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.cJoystick;
import static frc.robot.Constants.cXbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    

    public final Localization localization = new Localization();

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(localization);

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final ElevatorPositionCommand elevatorPositionCommand1 = new ElevatorPositionCommand(elevatorSubsystem, 0.01);
    public final ElevatorPositionCommand elevatorPositionCommand2 = new ElevatorPositionCommand(elevatorSubsystem, 0.5);

//    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private int counter;

    private final ShuffleboardTab kinematicsTab;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        SmartDashboard.putData(CommandScheduler.getInstance());

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem,new TeleopDriveCommand(swerveSubsystem));

        new Trigger(() -> cJoystick.getRawButton(3)).onTrue(elevatorPositionCommand1);
        new Trigger(() -> cJoystick.getRawButton(4)).onTrue(elevatorPositionCommand2);

        setXboxBindings();

        this.kinematicsTab = Shuffleboard.getTab("kinematics");
        this.kinematicsTab.addDouble("Xv", () -> this.swerveSubsystem.getOdoVel().vxMetersPerSecond);
        this.kinematicsTab.addDouble("Yv", () -> this.swerveSubsystem.getOdoVel().vyMetersPerSecond);
        this.kinematicsTab.addDouble("Hv", () -> this.swerveSubsystem.getOdoVel().omegaRadiansPerSecond);

        this.kinematicsTab.addDouble("Xa", () -> this.swerveSubsystem.getAcc().get().getX());
        this.kinematicsTab.addDouble("Ya", () -> this.swerveSubsystem.getAcc().get().getY());
        this.kinematicsTab.addDouble("Za", () -> this.swerveSubsystem.getAcc().get().getZ());

    }

    public void prePeriodic(boolean teleop) {

        if(teleop) {
            localization.move();
            localization.measure(swerveSubsystem);
            localization.updateField();
        }

    }

    public void postPeriodic() {


    }

    private void setXboxBindings() {
       //new Trigger(cXbox::getRightBumperButtonPressed).onTrue(new InstantCommand( () -> swerveSubsystem.offsetGyro()));

        new Trigger(cXbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        new Trigger(cXbox::getStartButtonPressed).onTrue(new InstantCommand(localization::reset));

        PathConstraints constraints = new PathConstraints(2, 4, Math.PI * 6, Math.PI * 6);

        new Trigger(cXbox::getAButton).whileTrue(Commands.sequence(
                (new PathToCommand(new Pose2d(7.28, 1.88, new Rotation2d(0)), 0, constraints, swerveSubsystem)),
                new AccuratePathCommand(new Pose2d(7.28, 1.88, new Rotation2d()), swerveSubsystem)));

        new Trigger(cXbox::getYButton).whileTrue(Commands.sequence(
                new PathToCommand(new Pose2d(7.3, 5.02, new Rotation2d(0)), 0, constraints, swerveSubsystem),
                new AccuratePathCommand(new Pose2d(7.3, 5.02, new Rotation2d()), swerveSubsystem)));

        new Trigger(cXbox::getBButton).whileTrue(Commands.sequence(
                new PathToCommand(new Pose2d(5.9, 3.18, new Rotation2d(Math.PI)), 0, constraints, swerveSubsystem),
                new AccuratePathCommand(new Pose2d(5.9, 3.18, new Rotation2d(Math.PI)), swerveSubsystem)));

        new Trigger(cXbox::getXButton).whileTrue(new RunCommand(()->swerveSubsystem.driveRobotOriented(new ChassisSpeeds(1,0,0)),swerveSubsystem));

        new Trigger(cXbox::getLeftBumperButtonPressed).onTrue(new AccuratePathCommand(
                new Pose2d(7.28, 1.88, new Rotation2d()),
                swerveSubsystem));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
//    public Command getAutonomousCommand()
//    {
//    }
}
