// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants{
    public static final int kLeftFrontMotorPort = 6;
    public static final int kRightFrontMotorPort = 5;
    public static final int kLeftBackMotorPort = 3;
    public static final int kRightBackMotorPort = 2;

    public static final int kLeftEncoderChannelA = 2;
    public static final int kLeftEncoderChannelB = 3;
    public static final int kRightEncoderChannelA = 0;
    public static final int kRightEncoderChannelB = 1;
    public static final double kEncoderTick2Feet = 1.0 / 2048.0 * Math.PI * 9 / 17.8;

    public static final double returnLimit = 0.6;
    public static final double DEAD_ZONE_THRESHOLD = 0.1;
    public static final double LEFT_MOTOR_CONTROLLER_DISTORTION = 0.1;
    public static final double RIGHT_MOTOR_CONTROLLER_DISTORTION = 1.2;

    public static final double kAutoDriveForwardDistance = -7;
    public static final double kAutoDriveForwardSpeed = 0.5;
  }

  public static final class Joystick {
    public static final int rightBtn = 6;
  }
}
