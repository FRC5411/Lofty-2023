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
        public static final int PDH_ID = 1;
        public static final int PCH_ID = 2;
        public static final int PIGEON_ID = 3;

        public static final int FL_CANCODER_ID = 4;
        public static final int FR_CANCODER_ID = 5;
        public static final int BL_CANCODER_ID = 6;
        public static final int BR_CANCODER_ID = 7;

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int FL_AZIMUTH_ID = 21;
        public static final int FR_AZIMUTH_ID = 22;
        public static final int BL_AZIMUTH_ID = 23;
        public static final int BR_AZIMUTH_ID = 24;

        public static final int ARM_STAGE_1_ID = 31;
        public static final int ARM_STAGE_1_FOLLOWER_ID = 32;
        public static final int ARM_STAGE_2_ID = 33;
        public static final int ARM_STAGE_3_ID = 34;

        public static final int GRIP_LEFT_ID = 41;
        public static final int GRIP_RIGHT_ID = 31;
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

    public class LL {
        public static final double SLOPE = 0;
        public static final double YINT = 0;
    }

    public class ARM {       
         public static final double JOINT_ANGLE_DEADZONE = 2;
         public static final double BISCEP_LENGTH = 20;        
         public static final double ELBOW_LENGTH = 30;
         public static final double LOW_ARM_ANG = 152.8;
         public static final double LOW_ELBOW_ANG = 168.9;
         public static final double LOW_CLAW_ANG = -14.5;
         public static final double HIGH_ARM_ANG = 83;
         public static final double HIGH_ELBOW_ANG = 168.9;
         public static final double HIGH_CLAW_ANG = -142.5;
         public static final double IDLE_ARM_ANG = 63.8;
         public static final double IDLE_ELBOW_ANG = -38.4;
         public static final double IDLE_CLAW_ANG = 96.9;
         public static final double FETCH_ARM_ANG = 0;
         public static final double FETCH_ELBOW_ANG = 0;
         public static final double FETCH_CLAW_ANG = 0;
 }

     public class POP {
         public static final double F = 0;
         public static final double R = 0;
         public static final double SPEED = 0.5;
     }

     public class LED {
         public static final double YELLOW = 0.69;
         public static final double PURPLE = 0.91;
         public static final double RED = 0.61;
         public static final double GREEN = 0.77;
         public static final double RAINBOW = -0.99;
     }
}