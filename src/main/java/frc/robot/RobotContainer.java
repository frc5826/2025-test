// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TestVarianceCommand;
import frc.robot.localization.Localization;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.xbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    

    public final Localization locatization = new Localization();

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(locatization);

    private int counter;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        SmartDashboard.putData(CommandScheduler.getInstance());

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem,new TeleopDriveCommand(swerveSubsystem));

        new Trigger(xbox::getAButton).whileTrue(new TestVarianceCommand(1,
                ss -> ss.getOdoVel().vxMetersPerSecond,
                s -> new ChassisSpeeds(s, 0, 0),
                swerveSubsystem));

        new Trigger(xbox::getBButton).whileTrue(new TestVarianceCommand(1,
                ss -> ss.getOdoVel().vyMetersPerSecond,
                s -> new ChassisSpeeds(0, s, 0),
                swerveSubsystem));

        new Trigger(xbox::getYButton).whileTrue(new TestVarianceCommand(1,
                ss -> ss.getOdoVel().omegaRadiansPerSecond,
                s -> new ChassisSpeeds(0, 0, s),
                swerveSubsystem));

        new Trigger(xbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        new Trigger(xbox::getStartButtonPressed).onTrue(new InstantCommand(locatization::reset));

    }

    public void prePeriodic() {
        locatization.move();
        locatization.measure(swerveSubsystem);
        locatization.updateField();
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
