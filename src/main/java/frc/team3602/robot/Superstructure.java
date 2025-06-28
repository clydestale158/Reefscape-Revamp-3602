package frc.team3602.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.subsystems.ElevSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static frc.team3602.robot.Constants.ElevConstants.*;

public class Superstructure {
    private final DrivetrainSubsystem driveSubsys;
    private final PivotSubsystem pivotSubsys;
    private final ElevSubsystem elevSubsys;
    private final CommandXboxController xbox;
    // private ClimberSubsystem climberSubsys;

    private final SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric teleopDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Superstructure(DrivetrainSubsystem driveSubsys, PivotSubsystem pivotSubsys, ElevSubsystem elevSubsys,
            CommandXboxController xbox) {
        this.driveSubsys = driveSubsys;
        this.pivotSubsys = pivotSubsys;
        this.elevSubsys = elevSubsys;
        this.xbox = xbox;
    }

    /**
     * command that runs the pivot intake until the sensor is triggered, compatable
     * with auton
     */
    public Command intakeCoral() {
        return pivotSubsys.runIntake(INTAKE_CORAL_SPEED).until(() -> pivotSubsys.sensorIsTriggered());
    }

    /**
     * command that runs the pivot intake until the !sensor is triggered, compatable
     * with auton
     */
    public Command outtakeCoral() {
        return pivotSubsys.runIntake(SCORE_CORAL_SPEED).until(() -> !pivotSubsys.sensorIsTriggered());
    }

    /** Command that runs the pivot intake in reverse for 0.2 seconds */
    public Command gripCoral() {
        return pivotSubsys.runIntake(0.1).withTimeout(0.2);
    }

    /** Command sequence, NOT compatable with auton */
    public Command getCoral() {
        return sequence(
                setElevator(ELEV_DOWN),
                setPivot(INTAKE_CORAL_ANGLE),
                intakeCoral()
        // gripCoral()
        );
    }

    /**
     * Command sequence to stow the pivot(and wait until isNearGoal), compatable
     * with auton
     */
    public Command setPivot(double angle) {
        return sequence(
                pivotSubsys.setAngle(angle),
                waitUntil(() -> pivotSubsys.isNearGoal()));
    }

    /**
     * command sequence to move the elevator after the pivot is stowed, compatable
     * with auton
     */
    public Command setElevator(double elevHeight) {
        return sequence(
                setPivot(STOW_ANGLE),
                elevSubsys.setHeight(elevHeight),
                waitUntil(() -> elevSubsys.isNearGoal()));
    }

    /** Command sequence, ONLY compatable with AUTON!! */
    public Command autoScoreCoral() {
        return sequence(
                pivotSubsys.setAngle(SCORE_CORAL_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(-0.3, 0, 0))).withTimeout(0.3),
                setElevator(ELEV_DOWN));
    }

    /** Command sequence, ONLY compatable with AUTON!! */
    public Command autoScoreCoralL4() {
        return sequence(
                setPivot(SCORE_CORAL_L4_ANGLE),
                outtakeCoral(),
                elevSubsys.setHeight(ELEV_L4_BUMP),
                waitUntil(() -> elevSubsys.isNearGoal()),
                driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(-0.3, 0, 0))).withTimeout(0.5),
                setElevator(ELEV_DOWN));

    }

    /** command sequence. ONLY compatable with TELEOP!! */
    public Command scoreCoral() {
        return sequence(
                setPivot(SCORE_CORAL_ANGLE),
                outtakeCoral(),
                driveSubsys.applyRequest(() -> teleopDrive.withVelocityX(-0.3)).withTimeout(0.5),
                setElevator(ELEV_DOWN));
    }

    /** command sequence. ONLY compatable with TELEOP!! */
    public Command scoreCoralL4() {
        return sequence(
                //setElevator(ELEV_L4),
                race(
                setPivot(SCORE_CORAL_L4_ANGLE),
                waitSeconds(0.6)),

                race(
                outtakeCoral(),
                waitSeconds(1)),

                pivotSubsys.setIntake(-0.5),
                elevSubsys.setHeight(ELEV_L4_BUMP),
                waitUntil(() -> elevSubsys.isNearGoal()),
                driveSubsys.applyRequest(() -> teleopDrive.withVelocityX(-0.3)).withTimeout(0.3),
                pivotSubsys.setIntake(0),
                setElevator(ELEV_DOWN));
    }

    public Command intakeAlgaeL2() {
        return sequence(
                setElevator(ELEV_L2_ALGAE),
                setPivot(INTAKE_ALGAE_ANGLE),
                pivotSubsys.intakeAlgae().until(() -> pivotSubsys.hasAlgae()));
    }

    public Command intakeAlgaeL3() {
        return sequence(
                setElevator(ELEV_L2_ALGAE),
                setPivot(INTAKE_ALGAE_ANGLE),
                pivotSubsys.intakeAlgae().until(() -> pivotSubsys.hasAlgae()));
    }

    public Command holdAlgae() {
        return sequence(
                pivotSubsys.setIntake(HOLD_ALGAE_SPEED),
                elevSubsys.setHeight(ELEV_DOWN));
    }

    /**AUTON compatable ONLY */
    public Command autoAlignLeft() {
        return driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(0.0, 0.6, 0.0)))
                .until(() -> !driveSubsys.seesLeftSensor());
    }

    /**AUTON compatable ONLY */
    public Command autoAlignRight() {
        return driveSubsys.applyRequest(() -> autoDrive.withSpeeds(new ChassisSpeeds(0.0, -0.75, 0.0)))
                .until(() -> !driveSubsys.seesRightSensor());
    }

    public Command down() {
        return Commands.sequence(
                sequence(
                        elevSubsys.setHeight(ELEV_L3),
                        waitUntil(elevSubsys::isNearGoal)).unless(() -> elevSubsys.getEncoder() < 28.0),

                pivotSubsys.setAngle(STOW_ANGLE),
                waitUntil(pivotSubsys::isNearGoal),

                elevSubsys.setHeight(0.1),

                pivotSubsys.setAngle(INTAKE_CORAL_ANGLE),
                waitUntil(pivotSubsys::isNearGoal),
                pivotSubsys.runIntake(0.1).until(pivotSubsys::sensorIsTriggered));
    }

    /** Command sequence, NOT compatable with auton */
    public Command removeAlgaeLow() {
        return sequence(
          
                        sequence(setPivot(INTAKE_ALGAE_ANGLE),
                                elevSubsys.setHeight(ELEV_L2_ALGAE),
                                pivotSubsys.runIntake(INTAKE_ALGAE_SPEED).until(() -> pivotSubsys.hasAlgae()),
                                waitSeconds(0.5),
                                pivotSubsys.setIntake(HOLD_ALGAE_SPEED))
                

                // driveSubsys
                //         .applyRequest(() -> teleopDrive.withVelocityX(-0.3 + xbox.getLeftY() * 0.4)
                //                 .withVelocityY(xbox.getLeftX() * 0.4).withRotationalRate(xbox.getRightX() * 0.2))
                //         .withTimeout(0.3),
                // elevSubsys.setHeight(ELEV_L1)
                );
    }

    /** Command sequence, NOT compatable with auton */
    public Command removeAlgaeHigh() {
        return sequence(
          setPivot(INTAKE_ALGAE_ANGLE),
                        elevSubsys.setHeight(ELEV_L3_ALGAE),
                        pivotSubsys.runIntake(INTAKE_ALGAE_SPEED).until(() -> pivotSubsys.hasAlgae()),
                        waitSeconds(0.5),
                        pivotSubsys.setIntake(HOLD_ALGAE_SPEED)
                // driveSubsys
                //         .applyRequest(() -> teleopDrive.withVelocityX(-0.3 + xbox.getLeftY() * 0.4)
                //                 .withVelocityY(xbox.getLeftX() * 0.4).withRotationalRate(xbox.getRightX() * 0.2))
                //         .withTimeout(0.3),
                // elevSubsys.setHeight(ELEV_L1)
                );
    }

    /** Run end command, NOT compatable with auton */
    public Command scoreAlgae() {
        return 
            sequence(
                setPivot(-65),
            pivotSubsys.setIntake(SCORE_ALGAE_SPEED),
            waitSeconds(0.3),
                    pivotSubsys.setIntake(0),
                    setElevator(ELEV_DOWN));
        
    }

    public Command placeAlgaeInBarge() {
        return Commands.sequence(
                pivotSubsys.setIntake(0),

                pivotSubsys.setAngle(22.0),
                waitUntil(pivotSubsys::isNearGoal),

                elevSubsys.setHeight(ELEV_L3),
                waitUntil(elevSubsys::isNearGoal),

                parallel(
                        sequence(
                                elevSubsys.setHeight(ELEV_L4),
                                waitUntil(elevSubsys::isNearGoal)),
                        sequence(
                                pivotSubsys.setIntake(-0.7),
                                pivotSubsys.setAngle(30.0))),
                waitSeconds(2.5),               
                pivotSubsys.setIntake(0));
    }

    public Command downFromBarge() {
        return sequence(
                setPivot(-30),
                elevSubsys.setHeight(ELEV_L2),
                waitSeconds(0.2),
                setElevator(ELEV_DOWN));
    }

    public Command storeAlgae() {
        return sequence(
                 driveSubsys
                        .applyRequest(() -> teleopDrive.withVelocityX(-0.3 + xbox.getLeftY() * 0.4)
                                .withVelocityY(xbox.getLeftX() * 0.4).withRotationalRate(xbox.getRightX() * 0.2))
                        .withTimeout(0.3),
                elevSubsys.setHeight(ELEV_L1),
                setPivot(HOLD_ALGAE_ANGLE));
    }
}
