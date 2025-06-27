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

        public final static int DRIVE_LEFT_LASER_ID = 54;
        public final static int DRIVE_RIGHT_LASER_ID = 55;//ids might be wrong, but this is functional lol

        // gearing and current limits
        public final static double ELEV_GEARING = 12;
        public final static double ELEV_CURRENT_LIMIT = 35;

        public final static double CLIMB_GEARING = 6;
        public final static double CLIMB_CURRENT_LIMIT = 30;

        public final static double PIVOT_GEARING = 12;
        public final static double PIVOT_CURRENT_LIMIT = 30;
    }

    public class PivotConstants {
        // intake speeds
        public final static double INTAKE_CORAL_SPEED = 0.5;// TODO check/fix irl
        public final static double SCORE_CORAL_SPEED = 0.5;// TODO check/fix irl
        public final static double INTAKE_ALGAE_SPEED = -0.5;
        public final static double HOLD_ALGAE_SPEED = -0.05;
        public final static double SCORE_ALGAE_SPEED = 0.3;

        // pivot angles
        public final static double INTAKE_CORAL_ANGLE = 110;// TODO check/fix irl
        public final static double STOW_ANGLE = 85;
        public final static double INTAKE_ALGAE_ANGLE = 110;// TODO check/fix irl

        public final static double SCORE_CORAL_ANGLE = 80;
        public final static double SCORE_CORAL_L4_ANGLE = 80;
    }

    public class ElevConstants {
        public final static double ELEV_DOWN = 0.0;
        public final static double ELEV_L1 = 2;
        public final static double ELEV_L2 = 6.5;
        public final static double ELEV_L3 = 16;
        public final static double ELEV_L4 = 31;
        public final static double ELEV_L4_BUMP = 33;

        public final static double ELEV_L2_ALGAE = 13;
        public final static double ELEV_L3_ALGAE = 22;
        public final static double ELEV_ALGAE_PROCESSOR = 2;
    }

}
