// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveTimeCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TestVarianceCommand;
import frc.robot.localization.Localization;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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

//    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private int counter;

    private final ShuffleboardTab kinematicsTab;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        SmartDashboard.putData(CommandScheduler.getInstance());

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem,new TeleopDriveCommand(swerveSubsystem));

        new Trigger(cXbox::getAButton).whileTrue(new TestVarianceCommand(1,
                ss -> ss.getOdoVel().vxMetersPerSecond,
                s -> new ChassisSpeeds(s, 0, 0),
                swerveSubsystem));

        new Trigger(cXbox::getBButton).whileTrue(new TestVarianceCommand(1,
                ss -> ss.getOdoVel().vyMetersPerSecond,
                s -> new ChassisSpeeds(0, s, 0),
                swerveSubsystem));

        new Trigger(cXbox::getYButton).whileTrue(new TestVarianceCommand(1,
                ss -> ss.getOdoVel().omegaRadiansPerSecond,
                s -> new ChassisSpeeds(0, 0, s),
                swerveSubsystem));

        new Trigger(cXbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        new Trigger(cXbox::getRightBumperButton).whileTrue(new MoveTimeCommand(
                1,
                new ChassisSpeeds(2, 0, 0),
                swerveSubsystem,
                localization));

        new Trigger(cXbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        new Trigger(cXbox::getStartButtonPressed).onTrue(new InstantCommand(localization::reset));


        this.kinematicsTab = Shuffleboard.getTab("kinematics");
        this.kinematicsTab.addDouble("Xv", () -> this.swerveSubsystem.getOdoVel().vxMetersPerSecond);
        this.kinematicsTab.addDouble("Yv", () -> this.swerveSubsystem.getOdoVel().vyMetersPerSecond);
        this.kinematicsTab.addDouble("Hv", () -> this.swerveSubsystem.getOdoVel().omegaRadiansPerSecond);

        this.kinematicsTab.addDouble("Xa", () -> this.swerveSubsystem.getAcc().get().getX());
        this.kinematicsTab.addDouble("Ya", () -> this.swerveSubsystem.getAcc().get().getY());
        this.kinematicsTab.addDouble("Za", () -> this.swerveSubsystem.getAcc().get().getZ());

    }

    public void prePeriodic(boolean teleop) {

//        if (counter++ % 1 == 0 && teleop) {
//            localization.move();
//            localization.measure(swerveSubsystem);
//            localization.updateField();
//        }

    }

    public void postPeriodic() {


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
