package frc.team3602.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.team3602.robot.Constants.HardwareConstants.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team3602.robot.subsystems.drive.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();


  // //TODO request FOUR lasers? could be fire...
  // private LaserCan leftInnerLaser = new LaserCan(DRIVE_LEFT_INNER_LASER_ID);
  // private LaserCan leftOuterLaser = new LaserCan(DRIVE_LEFT_OUTER_LASER_ID);
  // private LaserCan rightInnerLaser = new LaserCan(DRIVE_RIGHT_INNER_LASER_ID);
  // private LaserCan rightOuterLaser = new LaserCan(DRIVE_RIGHT_OUTER_LASER_ID);

  

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)),
          null,
          this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct
   * the devices themselves. If they need the devices, they can access them
   * through
   * getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules             Constants for each specific module
   */
  public DrivetrainSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct
   * the devices themselves. If they need the devices, they can access them
   * through
   * getters in the classes.
   *
   * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If
   *                                unspecified or set to 0 Hz, this is 250 Hz on
   *                                CAN FD, and 100 Hz on CAN 2.0.
   * @param modules                 Constants for each specific module
   */
  public DrivetrainSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct
   * the devices themselves. If they need the devices, they can access them
   * through
   * getters in the classes.
   *
   * @param drivetrainConstants       Drivetrain-wide constants for the swerve
   *                                  drive
   * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
   *                                  unspecified or set to 0 Hz, this is 250 Hz
   *                                  on
   *                                  CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry
   *                                  calculation
   *                                  in the form [x, y, theta]ᵀ, with units in
   *                                  meters
   *                                  and radians
   * @param visionStandardDeviation   The standard deviation for vision
   *                                  calculation
   *                                  in the form [x, y, theta]ᵀ, with units in
   *                                  meters
   *                                  and radians
   * @param modules                   Constants for each specific module
   */
  public DrivetrainSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve
   * drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
      });
    }

    distance = getDistanceFromReef();
    SmartDashboard.putNumber("LASER", distance);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // // NEW
  // // Pathplanner - create paths on the fly

  // // lists are the points the robot will use to travel along
  // // TODO make waypoints more accurate & fix with real robot testing
  // // I don't think it needs red vs blue, but in the meantime, I created some
  // red
  // // vars too
  // // in the var names, barge refers to the barge side of the field. from the
  // // drivers station, this is the left side of the field
  // private List<Waypoint> blueFarBargeReefToCoralIntake =
  // PathPlannerPath.waypointsFromPoses(
  // new Pose2d(5.7, 6.4, Rotation2d.fromDegrees(-10)),
  // new Pose2d(2.6, 7.3, Rotation2d.fromDegrees(10)),
  // flyPathPosesConstants.blueBargeCoralIntakePose);
  // private List<Waypoint> blueCloseBargeReefToCoralIntake =
  // PathPlannerPath.waypointsFromPoses(
  // new Pose2d(2.6, 7.3, Rotation2d.fromDegrees(10)),
  // flyPathPosesConstants.blueBargeCoralIntakePose);
  // private List<Waypoint> blueFarNONBargeReefToCoralIntake =
  // PathPlannerPath.waypointsFromPoses(
  // new Pose2d(5.7, 2.2, Rotation2d.fromDegrees(20)),
  // new Pose2d(2.6, 1.8, Rotation2d.fromDegrees(60)),
  // new Pose2d(1.2, 1.8, Rotation2d.fromDegrees(125)),

  // flyPathPosesConstants.blueNONBargeCoralIntakePose);
  // private List<Waypoint> blueCloseNONBargeReefToCoralIntake =
  // PathPlannerPath.waypointsFromPoses(
  // new Pose2d(2.6, 1.8, Rotation2d.fromDegrees(60)),
  // new Pose2d(1.2, 1.8, Rotation2d.fromDegrees(125)),
  // flyPathPosesConstants.blueNONBargeCoralIntakePose);

  // private PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 *
  // Math.PI, 4 * Math.PI);

  // // command to get us to the coral station
  // // we can use booleans to get our pose and try a path based on that(depending
  // on
  // // y, we can go to the other coral station)
  // public Command flypathToCoralStation() {
  // if (getState().Pose.getY() >= 4) {
  // if (getState().Pose.getX() >= 4.5) {
  // try {

  // // Load the path you want to follow using its name in the GUI
  // PathPlannerPath flypath = new PathPlannerPath(
  // blueFarBargeReefToCoralIntake,
  // constraints,
  // null, // The ideal starting state, this is only relevant for pre-planned
  // paths, so can
  // // be null for on-the-fly paths.
  // new GoalEndState(0.0, Rotation2d.fromDegrees(35)) // Goal end state. You can
  // set a holonomic
  // // rotation here. If using a differential
  // // drivetrain, the rotation will have no
  // // effect.
  // );
  // flypath.preventFlipping = true;

  // // Create a path following command using AutoBuilder.
  // return AutoBuilder.followPath(flypath);
  // } catch (Exception e) {
  // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  // return Commands.none();
  // }}else{
  // try{

  // // Load the path you want to follow using its name in the GUI
  // PathPlannerPath flypath = new PathPlannerPath(
  // blueCloseBargeReefToCoralIntake,
  // constraints,
  // null, // The ideal starting state, this is only relevant for pre-planned
  // paths, so can be null for on-the-fly paths.
  // new GoalEndState(0.0, Rotation2d.fromDegrees(35)) // Goal end state. You can
  // set a holonomic rotation here. If using a differential drivetrain, the
  // rotation will have no effect.
  // );
  // flypath.preventFlipping = true;

  // // Create a path following command using AutoBuilder.
  // return AutoBuilder.followPath(flypath);
  // } catch (Exception e) {
  // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  // return Commands.none();
  // }}}else if (getState().Pose.getY() < 4){
  // if(getState().Pose.getX()>= 4.5){
  // try{

  // // Load the path you want to follow using its name in the GUI
  // PathPlannerPath flypath = new PathPlannerPath(
  // blueFarNONBargeReefToCoralIntake,
  // constraints,
  // null, // The ideal starting state, this is only relevant for pre-planned
  // paths, so can be null for on-the-fly paths.
  // new GoalEndState(0.0, Rotation2d.fromDegrees(145)) // Goal end state. You can
  // set a holonomic rotation here. If using a differential drivetrain, the
  // rotation will have no effect.
  // );
  // flypath.preventFlipping = true;

  // // Create a path following command using AutoBuilder.
  // return AutoBuilder.followPath(flypath);
  // } catch (Exception e) {
  // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  // return Commands.none();
  // }}else{
  // try{

  // // Load the path you want to follow using its name in the GUI
  // PathPlannerPath flypath = new PathPlannerPath(
  // blueCloseNONBargeReefToCoralIntake,
  // constraints,
  // null, // The ideal starting state, this is only relevant for pre-planned
  // paths, so can be null for on-the-fly paths.
  // new GoalEndState(0.0, Rotation2d.fromDegrees(145)) // Goal end state. You can
  // set a holonomic rotation here. If using a differential drivetrain, the
  // rotation will have no effect.
  // );
  // flypath.preventFlipping = true;

  // // Create a path following command using AutoBuilder.
  // return AutoBuilder.followPath(flypath);
  // } catch (Exception e) {
  // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  // return Commands.none();
  // }
  // }} else{
  // return Commands.none();
  // }
  // }

  public double getDistanceFromReef() {
    if (RobotBase.isSimulation()) {
      return 12;// TODO change lol
    } else {
      return (alignmentLASER.getMeasurement().distance_mm / 1000.0);
    }
  }

  public boolean seesReef(){
    if (RobotBase.isSimulation()) {
      return false;// TODO change lol
    } else {
      return (alignmentLASER.getMeasurement().distance_mm) < 1000;
    }
  }

  /**
   * Config for pathplanner's auto builder. MUST be called only AFTER named
   * commands are registered/configured
   */
  public SendableChooser<Command> autoChooser;

  public void configAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          () -> getState().Pose,
          this::resetPose,
          () -> getState().Speeds,
          (speeds, feedforwards) -> setControl(
              m_pathApplyRobotSpeeds.withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              new PIDConstants(10, 0, 0),
              new PIDConstants(7, 0, 0)),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);

      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      //autoChooser.setDefaultOption("null", print("You skibidi messed up big time, pure buns brat behavior"));
    } catch (Exception ex) {
      DriverStation.reportError("Pathplanner auto builder config FAILED", ex.getStackTrace());
      SmartDashboard.putString("Errors", "Pathplanner auto builder config FAILED");
    }
  }
}