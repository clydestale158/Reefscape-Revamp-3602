package frc.team3602.robot;

public class Constants {
    public class HardwareConstants {
        // Can ids 40-51 are used in the drivetrain
        public final static int ELEV_LEAD_MOTOR_ID = 53;
        public final static int ELEV_FOLLOW_MOTOR_ID = 57;
        public final static int PIVOT_MOTOR_ID = 52;
        public final static int INTAKE_MOTOR_ID = 56;
        public final static double CLIMBER_MOTOR_ID = 18;
        public final static int PIVOT_CANCODER_ID = 34;

        public final static int INTAKE_LASER_ID = 31;

        public final static int DRIVE_LEFT_LASER_ID = 55;
        public final static int DRIVE_RIGHT_LASER_ID = 54;

        // gearing and current limits
        public final static double ELEV_GEARING = 12;
        public final static double ELEV_CURRENT_LIMIT = 35;

       //TODO: find actual Gearing for Climb and a proper Current Limit
        public final static double CLIMB_GEARING = 6;
        public final static double CLIMB_CURRENT_LIMIT = 30;

        public final static double PIVOT_GEARING = 12;
        public final static double PIVOT_CURRENT_LIMIT = 30;
    }

    public class PivotConstants {
        // intake speeds
        public final static double INTAKE_CORAL_SPEED = 0.5;// TODO check/fix irl
        public final static double SCORE_CORAL_SPEED = 0.5;// TODO check/fix irl
        public final static double INTAKE_ALGAE_SPEED = -0.5;// TODO check/fix irl
        public final static double HOLD_ALGAE_SPEED = 0.0;// TODO check/fix irl

        // pivot angles
        public final static double INTAKE_CORAL_ANGLE = 110;// TODO check/fix irl
        public final static double STOW_ANGLE = 80;// TODO check/fix irl
        public final static double INTAKE_ALGAE_ANGLE = 110;// TODO check/fix irl

        public final static double SCORE_CORAL_ANGLE = 80;// TODO check/fix irl
        public final static double SCORE_CORAL_L4_ANGLE = 70;// TODO check/fix irl
    }

    public class ElevConstants {
        public final static double ELEV_DOWN = 0.0;
        public final static double ELEV_L1 = 2;
        public final static double ELEV_L2 = 6.5;
        public final static double ELEV_L3 = 16;
        public final static double ELEV_L4 = 31;
        public final static double ELEV_L4_BUMP = 21;// TODO check/fix irl

        public final static double ELEV_L2_ALGAE = 13;
        public final static double ELEV_L3_ALGAE = 22;
        public final static double ELEV_ALGAE_PROCESSOR = 2;// TODO test irl
    }

    public final class ClimberConstants {
        public static final int motorCANId = 18;
        public static final double percentVoltageScalar = 0.4; // 0.2

         // Climb angles
         public final static double ATTACH_ANGLE = 110;// TODO check/fix irl
         public final static double CLIMBING_ANGLE = 110;// TODO check/fix irl
 
        // // Climber PID constants
        // public static final double KP = .1;
        // public static final double KI = 0;
        // public static final double KD = .1;

        // // Climber FFE
        // public static final double KS = .1;
        // public static final double KG = .1;
        // public static final double KV = .1;
        // public static final double KA = .1;

        // // Sim Constants
        // public static final int gearing = 1;
        // public static final double lengthMeters = 1;
        // public static final double massKG = 1;

        // // Sim climber PID constants
        // public static final double simKP = .1;
        // public static final double simKI = 0;
        // public static final double simKD = .1;

        // // SIm climber FFE constants
        // public static final double simKS = .1;
        // public static final double simKG = .1;
        // public static final double simKV = .1;
        // public static final double simKA = .1;
    }
}
