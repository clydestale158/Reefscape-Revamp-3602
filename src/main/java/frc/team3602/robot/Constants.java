package frc.team3602.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class HardwareConstants {
        public final static int ELEV_LEAD_MOTOR_ID = 53;
        public final static int ELEV_FOLLOW_MOTOR_ID = 57;
        public final static int PIVOT_MOTOR_ID = 52;
        public final static int INTAKE_MOTOR_ID = 56;
        public final static int PIVOT_ENCODER_ID = 34;
        public final static int INTAKE_LASER_ID = 31;

        // public final static int DRIVE_LEFT_INNER_LASER_ID = 0;// TODO check
        // public final static int DRIVE_LEFT_OUTER_LASER_ID = 0;// TODO check
        // public final static int DRIVE_RIGHT_INNER_LASER_ID = 0;// TODO check
        // public final static int DRIVE_RIGHT_OUTER_LASER_ID = 0;// TODO check

        public final static int DRIVE_LASER_ID = 0;// TODO check
        public final static int PIVOT_CANCODER_ID = 1; // TODO check

        public final static double ELEV_GEARING = 48;// TODO check
        public final static double ELEV_CURRENT_LIMIT = 35;

        public final static double PIVOT_GEARING = 48;// TODO check
        public final static double PIVOT_CURRENT_LIMIT = 30;

    }

    public class PivotConstants{
        public final static double INTAKE_SPEED = 0.5;//TODO check/fix irl
        public final static double SCORE_SPEED = 0.5;//TODO check/fix irl
        public final static double INTAKE_ALGAE_SPEED = 0.5;//TODO check/fix irl
        public final static double HOLD_ALGAE_SPEED = 0.5;//TODO check/fix irl


        public final static double INTAKE_ANGLE = 110;//TODO check/fix irl
        public final static double STOW_ANGLE = 80;//TODO check/fix irl
        public final static double INTAKE_ALGAE_ANGLE = 110;//TODO check/fix irl


        public final static double SCORE_ANGLE = 80;//TODO check/fix irl
        public final static double SCORE_L4_ANGLE = 70;//TODO check/fix irl
    }

    public class ElevConstants{
        public final static double ELEV_DOWN = 0.0;
        public final static double ELEV_L2 = 8;//TODO check/fix irl
        public final static double ELEV_L3 = 14;//TODO check/fix irl
        public final static double ELEV_L4 = 20;//TODO check/fix irl
        public final static double ELEV_L4_BUMP = 21;//TODO check/fix irl

        public final static double ELEV_L2_ALGAE = 8;//TODO check/fix irl
        public final static double ELEV_L3_ALGAE = 14;//TODO check/fix irl

    }

    public class VisionConstants {
        public final static String FRONT_CAM_NAME = "Front Cam";
        public final static int FRONT_CAM_WIDTH_RES = 4656;
        public final static int FRONT_CAM_HEIGHT_RES = 3496;
        public final static double FRONT_CAM_FOV_DEG = 90;
        public final static Transform3d FRONT_CAM_TO_ROBOT = new Transform3d(
                new Translation3d(-.254, 0.254, 0.15),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));// TODO change

        public final static String BACK_CAM_NAME = "Back Cam";
        public final static int BACK_CAM_WIDTH_RES = 4656;
        public final static int BACK_CAM_HEIGHT_RES = 3496;
        public final static double BACK_CAM_FOV_DEG = 90;
        public final static Transform3d BACK_CAM_TO_ROBOT = new Transform3d(
                new Translation3d(0.38, -0.35, 0.178),
                new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(0.0)));// TODO change

    }
}
