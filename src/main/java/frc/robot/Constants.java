// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double cMaxVelocity = 5.05968; //DON'T CHANGE!!!!!!!!!!!!!!!!!!!!!!!
    public static final double cMaxAngularVelocity = 5;

    public static final XboxController cXbox = new XboxController(1);

    public static final double cTeleDriveDeadband = 0.2;
    public static final double cTurnDeadband = 0.8;

    public static class Elevator {
        public static final double cElevatorP = 0;
        public static final double cElevatorI = 0;
        public static final double cElevatorD = 0;
        public static final double cElevatorMinOutput = 0;
        public static final double cElevatorMaxOutput = 0;
        public static final double cElevatorV = 0;
        public static final double cElevatorG = 0;
        public static final double cElevatorMaxAcceleration = 0;
        public static final double cElevatorMaxVelocity = 0;

        public static final double cElevatorToEncoderConversion = 0;//TODO find actual value
        public static final double cElevatorToMotorConversion = 0;//TODO find actual value

        public static final double cElevatorHeightMin = 0;//TODO
        public static final double cElevatorHeightMax = 0;//TODO

        public static final double cElevatorDeadband = 0;

    }

    public static class Coralizer {

        public static final double cWristMaxAngle = 0; //TODO
        public static final double cWristMinAngle = 0; //TODO
        public static final double cMotorToRadians = 0;
        public static final double cWristOffset = 0;
        public static final double cWristV = 0;
        public static final double cWristG = 0;
        public static final double cWristMaxVelocity = 0;
        public static final double cWristMaxAcceleration = 0;
        public static final double cWristMinOutput = 0;
        public static final double cWristMaxOutput = 0;
        public static final double cWristDeadband = 0;

    }
}
