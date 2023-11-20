package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class DriveVars {
    public static class DriveConstants {
        // robot width (meters)
        public static final double ROBOT_WIDTH_METERS = 0.6858;
        // wheel diameter (meters)
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double WHEEL_PERIMETER_METERS = WHEEL_DIAMETER_METERS * Math.PI;
            
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double AZIMUTH_GEAR_RATIO = 12.8;
    
        // encoder offsets (degrees)
        public static final double FL_ECODER_OFFSET = -313.506 + 0.5;
        public static final double FR_ECODER_OFFSET = -69.082 + 0.5;
        public static final double BL_ECODER_OFFSET = -45.791 + 180;
        public static final double BR_ECODER_OFFSET = -257.783;
    
            /** maximum strafe speed (meters per second) */
        public static final double MAX_LINEAR_SPEED = 5.4;
            /** maximum rotation speed (radians per second) */
        public static final double MAX_ROTATION_SPEED = Math.PI * 2;
        public static final double SWERVE_SLOW_SPEED_PERCENTAGE = 0.1;
        public static final double ROTATION_SCALE_FACTOR = 0.65;
    
        // pid values
        public static final double AZIMUTH_kP = 0.0105;// 0.0115//0.0125;//0.025 //0.05//0.1 //0.01 //0.0053 sds: 0.2;
                                                           // rylan: 0.65
        public static final double AZIMUTH_kD = 0.000265;// 0.000275;//0.0003;//0.0004;//0.0005;//0.0006;//0.0006125;//0.0006125//0.000625//0.00065//0.0006;//0.00055//0.0005;//0.002//0.001//0.00075
                                                             // //0.0005;//0.00025
        public static final double AZIMUTH_kF = 0.04;// 0.05
        public static final double AZIMUTH_DEADBAND = 0.06;// 0.1;//0.06;//0.075over slop;//0.1Over slop//0.05 under
                                                               // slop
    
        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.088062; // 0.04;//0.07;//0.06; //0.044057
        public static final double DRIVE_kF = 0.028998;// 0.04//0.06; //0.028998
    
        /* Maximum distance for a valid waypoint (meters) */
        public static final double MAX_WAYPOINT_DISTANCE = 0.5;
    
        public static final double SHWERVE_DRIVE_Kp = 0.044057;
        public static final double SHWERVE_DRIVE_Kd = 0;
    
        public static final double AUTO_BALANCE_Kp = 0.1;
        public static final double AUTO_BALANCE_Kd = 0;

        public static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = 
            new StatorCurrentLimitConfiguration(
                true, 
          60, 
          60, 
          0);
    
        public static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = 
            new StatorCurrentLimitConfiguration(
          true, 
          30, 
          40, 
          0.2);
    }
}
