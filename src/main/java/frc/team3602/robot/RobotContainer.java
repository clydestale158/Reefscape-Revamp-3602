package frc.team3602.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static frc.team3602.robot.Constants.ElevConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.simulation.Simulation;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.drive.generated.TunerConstants;
import frc.team3602.robot.subsystems.elevator.ElevSubsystem;
import frc.team3602.robot.subsystems.pivot.PivotSubsystem;
import frc.team3602.robot.vision.Vision;

public class RobotContainer {
    private CommandJoystick joystick;
    private CommandJoystick joystick2;
    private CommandXboxController xbox;
    @SuppressWarnings("unused")
    private CommandXboxController xbox2;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max
                                                                                      // angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric teleopDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final DrivetrainSubsystem drivetrain = TunerConstants.createDrivetrain();
    private final ElevSubsystem elevSubsystem = new ElevSubsystem();
    private PivotSubsystem pivotSubsystem;// = new PivotSubsystem(joystick);
    private Superstructure superstructure;// = new Superstructure(drivetrain, pivotSubsystem, elevSubsystem);
    private Simulation simulation;

    private final Vision vision = new Vision();

    public SendableChooser<Command> autoChooser;
    private final SendableChooser<Double> polarityChooser = new SendableChooser<>();

    public RobotContainer() {

        // SmartDashboard.putData(autoChooser);
        SmartDashboard.putData("Polarity", polarityChooser);
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);

        if (Utils.isSimulation()) {
            joystick = new CommandJoystick(0);
            joystick2 = new CommandJoystick(1);
            pivotSubsystem = new PivotSubsystem(joystick);
            superstructure = new Superstructure(drivetrain, pivotSubsystem, elevSubsystem);
            simulation = new Simulation(elevSubsystem, pivotSubsystem);
            configSimButtonBindings();

            vision.reset();
        } else {
            xbox = new CommandXboxController(0);
            xbox2 = new CommandXboxController(1);

            // configButtonBindings();
        }

        // TODO instiantiate pivot subsys and superstructure!!!!

        configDefaultCommands();
        configNamedCommands();
        drivetrain.configAutoBuilder();
        autoChooser = drivetrain.autoChooser;

    }

    private void configDefaultCommands() {
        if (Utils.isSimulation()) {
            drivetrain.setDefaultCommand(drivetrain.applyRequest(
                    () -> drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed * polarityChooser.getSelected())
                            .withVelocityY(joystick.getRawAxis(1) * MaxSpeed * polarityChooser.getSelected())
                            .withRotationalRate(-joystick2.getRawAxis(0) * MaxAngularRate)));
        } else {
            drivetrain.setDefaultCommand(drivetrain
                    .applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed * polarityChooser.getSelected())
                            .withVelocityY(xbox.getLeftX() * MaxSpeed * polarityChooser.getSelected())
                            .withRotationalRate(-xbox.getRightX() * MaxAngularRate)));
        }
    }

    private void configSimButtonBindings() {
        joystick.button(1).onTrue(pivotSubsystem.setAngle(-80));
        joystick.button(2).onTrue(pivotSubsystem.setAngle(0));
        joystick.button(3).onTrue(pivotSubsystem.setAngle(110));
        joystick.button(4).whileTrue(pivotSubsystem.runIntake(1));

        // joystick.button(1).onTrue(superstructure.setElevator(ELEV_DOWN));
        // joystick.button(2).onTrue(superstructure.scoreCoralL2());
        // joystick.button(3).onTrue(superstructure.scoreCoralL3());
        // joystick.button(4).onTrue(superstructure.scoreCoralL4());

        joystick2.button(1).onTrue(elevSubsystem.setHeight(0.05));
        joystick2.button(2).onTrue(elevSubsystem.setHeight(0.5));
        joystick2.button(3).onTrue(elevSubsystem.setHeight(1.0));
        joystick2.button(4).onTrue(elevSubsystem.setHeight(1.5));

    }

    private void configButtonBindings() {
        xbox.a().onTrue(superstructure.setElevator(ELEV_DOWN));
        xbox.b().onTrue(superstructure.scoreCoralL2());
        xbox.x().onTrue(superstructure.scoreCoralL3());
        xbox.y().onTrue(superstructure.scoreCoralL4());

        xbox.povDown().onTrue(superstructure.intakeCoral());
        xbox.povUp().onTrue(superstructure.outtakeCoral());

        xbox.povRight().whileTrue(
                drivetrain.applyRequest(() -> teleopDrive.withVelocityX(-0.3)).until(() -> drivetrain.seesReef()));
        xbox.povLeft().whileTrue(
                drivetrain.applyRequest(() -> teleopDrive.withVelocityX(0.3)).until(() -> drivetrain.seesReef()));

    }

    private void configNamedCommands() {
        NamedCommands.registerCommand("Test", print("Auton test"));
        NamedCommands.registerCommand("Intake Coral", superstructure.intakeCoral());
        NamedCommands.registerCommand("Outtake Coral", superstructure.outtakeCoral());

        NamedCommands.registerCommand("Set Elevator Down", superstructure.setElevator(ELEV_DOWN));
        NamedCommands.registerCommand("Set Elevator L2", superstructure.setElevator(ELEV_L2));
        NamedCommands.registerCommand("Set Elevator L3", superstructure.setElevator(ELEV_L3));
        NamedCommands.registerCommand("Set Elevator L4", superstructure.setElevator(ELEV_L4));

        NamedCommands.registerCommand("Score Coral", superstructure.autoScoreCoral());
        NamedCommands.registerCommand("Score Coral L4", superstructure.autoScoreCoralL4());
    }

    public void updateVision() {
        vision.updateViz(drivetrain.getState().Pose);
        vision.updateDashboard();
    }

}
