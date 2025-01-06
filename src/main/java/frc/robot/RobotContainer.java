// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.localization.Localization;
import frc.robot.subsystems.SwerveSubsystem;

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
    }

    public void prePeriodic() {
        locatization.move();
        locatization.measure(swerveSubsystem);
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
