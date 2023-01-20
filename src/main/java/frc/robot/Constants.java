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
    public class CAN {
        //CAN Bus IDs
        public static final int PIGEON_ID = 0;

        public static final int FL_CANCODER_ID = 5;
        public static final int FR_CANCODER_ID = 6;
        public static final int BL_CANCODER_ID = 7;
        public static final int BR_CANCODER_ID = 8;

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int FL_AZIMUTH_ID = 21;
        public static final int FR_AZIMUTH_ID = 22;
        public static final int BL_AZIMUTH_ID = 23;
        public static final int BR_AZIMUTH_ID = 24;
    }

    public class DRIVETRAIN {
        // robot width (meters)
        public static final double ROBOT_WIDTH = 0.6858;
        // wheel diameter (meters)
        public static final double WHEEL_DIAMETER = 0.1016;
        // drive gear ratio
        public static final double DRIVE_GEAR_RATIO = 6.75;

        // encoder offsets (degrees)
        public static final double FL_ECODER_OFFSET = -313.682;
        public static final double FR_ECODER_OFFSET = -166.553;
        public static final double BL_ECODER_OFFSET = -246.006;
        public static final double BR_ECODER_OFFSET = -204.258;
    
        
        /** maximum strafe speed (meters per second) */
        public static final double MAX_LINEAR_SPEED = 5.4;
        /** maximum rotation speed (radians per second) */
        public static final double MAX_ROTATION_SPEED = Math.PI*2;
            
        // pid values
        public static final double AZIMUTH_kP = 0.2;
        public static final double AZIMUTH_kD = 0.1;

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.044057;
        public static final double DRIVE_kF = 0.028998;
    }

    public static final double BEAM_BALANACED_DRIVE_KP =   1;
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
}
