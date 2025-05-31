package frc.team3602.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.elevator.ElevSubsystem;
import frc.team3602.robot.subsystems.pivot.PivotSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static frc.team3602.robot.Constants.ElevConstants.*;



public class Superstructure {
    private DrivetrainSubsystem driveSubsys;
    private PivotSubsystem pivotSubsys;
    private ElevSubsystem elevSubsys;

    private SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveRequest.RobotCentric teleopDrive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Superstructure(DrivetrainSubsystem driveSubsys, PivotSubsystem pivotSubsys, ElevSubsystem elevSubsys){
        this.driveSubsys = driveSubsys;
        this.pivotSubsys = pivotSubsys;
        this.elevSubsys = elevSubsys;
    }

    /**command that runs the pivot intake until the sensor is triggered */
    public Command intakeCoral(){
        return pivotSubsys.runIntake(INTAKE_SPEED).until(()-> pivotSubsys.sensorIsTriggered());
    }

        /**command that runs the pivot intake until the !sensor is triggered */
        public Command outtakeCoral(){
            return pivotSubsys.runIntake(SCORE_SPEED).until(()-> !pivotSubsys.sensorIsTriggered());
        }


    /**command sequence. NOT compatable with auton */
    public Command scoreCoralL2(){
        return sequence(
            pivotSubsys.setAngle(STOW_ANGLE),
            none().until(()-> pivotSubsys.isNearGoal()),
            elevSubsys.setHeight(ELEV_L2),
            none().until(() -> elevSubsys.isNearGoal()),
            pivotSubsys.setAngle(SCORE_ANGLE),
            outtakeCoral(),
            driveSubsys.applyRequest(()-> teleopDrive.withVelocityX(-0.3)).withTimeout(0.5),
            pivotSubsys.setAngle(STOW_ANGLE),
            none().until(()-> pivotSubsys.isNearGoal()),
            elevSubsys.setHeight(ELEV_DOWN)
            );
    }

        /**command sequence. ONLY compatable with auton */
        public Command autoScoreCoralL2(){
            return sequence(
                pivotSubsys.setAngle(STOW_ANGLE),
                none().until(()-> pivotSubsys.isNearGoal()),
                elevSubsys.setHeight(ELEV_L2),
                none().until(() -> elevSubsys.isNearGoal()),
                pivotSubsys.setAngle(SCORE_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(()-> autoDrive.withSpeeds(new ChassisSpeeds(-0.3, 0, 0))).withTimeout(0.5),
                pivotSubsys.setAngle(STOW_ANGLE),
                none().until(()-> pivotSubsys.isNearGoal()),
                elevSubsys.setHeight(ELEV_DOWN)
                );
        }

        /**command sequence. NOT compatable with auton */
        public Command scoreCoralL3(){
            return sequence(
                pivotSubsys.setAngle(STOW_ANGLE),
                none().until(()-> pivotSubsys.isNearGoal()),
                elevSubsys.setHeight(ELEV_L3),
                none().until(() -> elevSubsys.isNearGoal()),
                pivotSubsys.setAngle(SCORE_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(()-> teleopDrive.withVelocityX(-0.3)).withTimeout(0.5),
                pivotSubsys.setAngle(STOW_ANGLE),
                none().until(()-> pivotSubsys.isNearGoal()),
                elevSubsys.setHeight(ELEV_DOWN)
                );
        }
    
            /**command sequence. ONLY compatable with auton */
            public Command autoScoreCoralL3(){
                return sequence(
                    pivotSubsys.setAngle(STOW_ANGLE),
                    none().until(()-> pivotSubsys.isNearGoal()),
                    elevSubsys.setHeight(ELEV_L3),
                    none().until(() -> elevSubsys.isNearGoal()),
                    pivotSubsys.setAngle(SCORE_ANGLE),
                    outtakeCoral(),
                    driveSubsys.applyRequest(()-> autoDrive.withSpeeds(new ChassisSpeeds(-0.3, 0, 0))).withTimeout(0.5),
                    pivotSubsys.setAngle(STOW_ANGLE),
                    none().until(()-> pivotSubsys.isNearGoal()),
                    elevSubsys.setHeight(ELEV_DOWN)
                    );
            }
}
