// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final double maxVelocity = 3;
    public static final double maxAngularVelocity = 5;

    public static final XboxController xbox = new XboxController(1);

    public static final double teleDriveDeadband = 0.2;
    public static final double turnDeadband = 0.8;

    public static final PIDConstants drivePID = new PIDConstants(4, 0, 0);
    public static final PIDConstants turnPID = new PIDConstants(1, 0, 0);

}
