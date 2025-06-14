package frc.team3602.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.Climb.ClimberSubsystem;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.elevator.ElevSubsystem;
import frc.team3602.robot.subsystems.elevator.WeirdElevSubsystem;
import frc.team3602.robot.subsystems.pivot.PivotSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static frc.team3602.robot.Constants.ElevConstants.*;

public class Superstructure {
    private final DrivetrainSubsystem driveSubsys;
    private final PivotSubsystem pivotSubsys;
    private final ElevSubsystem elevSubsys;
    //private ClimberSubsystem climberSubsys;

    private final SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric teleopDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Superstructure(DrivetrainSubsystem driveSubsys, PivotSubsystem pivotSubsys, ElevSubsystem elevSubsys) {
        this.driveSubsys = driveSubsys;
        this.pivotSubsys = pivotSubsys;
        this.elevSubsys = elevSubsys;
    }

    /** command that runs the pivot intake until the sensor is triggered, compatable with auton */
    public Command intakeCoral() {
        return pivotSubsys.runIntake(INTAKE_SPEED).until(() -> pivotSubsys.sensorIsTriggered());
    }

    /** command that runs the pivot intake until the !sensor is triggered, compatable with auton */
    public Command outtakeCoral() {
        return pivotSubsys.runIntake(SCORE_SPEED).until(() -> !pivotSubsys.sensorIsTriggered());
    }

    /**Command sequence to stow the pivot(and wait until isNearGoal), compatable with auton */
    public Command setPivot(double angle) {
        return sequence(
                pivotSubsys.setAngle(angle),
                none().until(() -> pivotSubsys.isNearGoal()));
    }

    /**
     * command sequence to move the elevator after the pivot is stowed, compatable
     * with auton
     */
    public Command setElevator(double elevHeight) {
        return sequence(
                setPivot(STOW_ANGLE),
                elevSubsys.setHeight(elevHeight),
                none().until(()-> elevSubsys.isNearGoal()));
    }

    /**Command sequence, ONLY compatable with AUTON!! */
    public Command autoScoreCoral() {
        return sequence(
                pivotSubsys.setAngle(SCORE_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(-0.3, 0, 0))).withTimeout(0.5));
    }

    /**Command sequence, ONLY compatable with AUTON!! */
    public Command autoScoreCoralL4(){
        return sequence(
            setPivot(SCORE_L4_ANGLE),
            outtakeCoral(),
            elevSubsys.setHeight(ELEV_L4_BUMP),
            none().until(()-> elevSubsys.isNearGoal()),
            driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(-0.3, 0, 0))).withTimeout(0.5)
        );
    }

    /** command sequence. ONLY compatable with TELEOP*/
    public Command scoreCoralL2() {
        return sequence(
                setElevator(ELEV_L2),
                setPivot(SCORE_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(() -> teleopDrive.withVelocityX(-0.3)).withTimeout(0.5),
                setElevator(ELEV_DOWN));
    }

    /** command sequence. ONLY compatable with TELEOP!! */
    public Command scoreCoralL3() {
        return sequence(
                setElevator(ELEV_L3),
                setPivot(SCORE_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(() -> teleopDrive.withVelocityX(-0.3)).withTimeout(0.5),
                setElevator(ELEV_DOWN));
    }

    /** command sequence. ONLY compatable with TELEOP!! */
    public Command scoreCoralL4(){
        return sequence(
            setElevator(ELEV_L4),
            setPivot(SCORE_L4_ANGLE),
            outtakeCoral(),
            elevSubsys.setHeight(ELEV_L4_BUMP),
            none().until(()-> elevSubsys.isNearGoal()),
            driveSubsys.applyRequest(() -> teleopDrive.withVelocityX(-0.3)).withTimeout(0.5),
            setElevator(ELEV_DOWN)
        );
    }

    public Command intakeAlgaeL2(){
        return sequence(
            setElevator(ELEV_L2_ALGAE),
            setPivot(INTAKE_ALGAE_ANGLE),
            pivotSubsys.intakeAlgae().until(()-> pivotSubsys.hasAlgae())
        );
    }
 
    public Command intakeAlgaeL3(){
        return sequence(
            setElevator(ELEV_L2_ALGAE),
            setPivot(INTAKE_ALGAE_ANGLE),
            pivotSubsys.intakeAlgae().until(()-> pivotSubsys.hasAlgae())
        );
    }

    public Command autoAlignLeft() {
        return Commands.sequence(
                driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(0.0, 0.6, 0.0)))
                        .until(() -> !driveSubsys.seesLeftSensor())
        );
    }

    public Command autoAlignRight() {
        return Commands.sequence(
                driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(0.0, -0.75, 0.0)))
                        .until(() -> !driveSubsys.seesRightSensor())
        );
    }
}
