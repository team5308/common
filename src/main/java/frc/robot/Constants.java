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
    //SwerveModuleConstants
    public static final double kDriveP = 15.0;
    public static final double kDriveI = 0.01;
    public static final double kDriveD = 0.1;
    public static final double kDriveF = 0.2;

    public static final double kAngleP = 0.4;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;

    public static final double kAngleEncoderTicksPerRotation = 2048;
    public static final double kCANCoderTicksPerRotation = 4096;
    public static final double kEncoderGearRatio = (60.0*32.0/(15.0*10.0)); //60??40??
    // public static final double kCANCoderGearRatio = 1;
    public static final double kDriveGearRatio = 6.86;


    //In degrees
    public static final double LEFT_FRONT_ANGLE_OFFSET = 0;
    public static final double RIGHT_FRONT_ANGLE_OFFSET = 0;
    public static final double LEFT_BACK_ANGLE_OFFSET = 0;
    public static final double RIGHT_BACK_ANGLE_OFFSET = 0;

    //In meter
    public static final double TRACKWIDTH = 0.59;
    public static final double WHEELBASE = 0.59;

    public static final int RIGHT_FRONT_ANGLE_CAN = 11;
    public static final int LEFT_FRONT_ANGLE_CAN = 21;
    public static final int LEFT_BACK_ANGLE_CAN = 31;
    public static final int RIGHT_BACK_ANGLE_CAN = 41;

    public static final int RIGHT_FRONT_DRIVE_CAN = 12;
    public static final int LEFT_FRONT_DRIVE_CAN = 22;
    public static final int LEFT_BACK_DRIVE_CAN = 32;
    public static final int RIGHT_BACK_DRIVE_CAN = 42;



}
