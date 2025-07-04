package frc.team3602.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static frc.team3602.robot.Constants.ElevConstants.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.simulation.Simulation;
import frc.team3602.robot.subsystems.ElevSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    private CommandJoystick joystick;
    private CommandJoystick joystick2;
    private CommandXboxController xbox;
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
    @SuppressWarnings("unused")
    private Simulation simulation;

    private enum Driver {
        Karter,
        Cooper,
        Walter,
        Caroline,
        David,
        Riley,
        Jaxson,
        Laura,
        Cece,
        Abe_Perryham,
        Kennedy,
        Robert,
        Mentor,
        Other
    }

    public SendableChooser<Command> autoChooser;
    private final SendableChooser<Double> polarityChooser = new SendableChooser<>();
    private final SendableChooser<Driver> driverChooser = new SendableChooser<>();

    private double driverSpeedMultiplier = 1;

    public RobotContainer() {

        // SmartDashboard.putData(autoChooser);
        SmartDashboard.putData("Polarity", polarityChooser);
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);

        if (Utils.isSimulation()) {
            joystick = new CommandJoystick(0);
            joystick2 = new CommandJoystick(1);
            pivotSubsystem = new PivotSubsystem(joystick);
            superstructure = new Superstructure(drivetrain, pivotSubsystem, elevSubsystem, xbox);
            simulation = new Simulation(elevSubsystem, pivotSubsystem);
            configSimButtonBindings();

        } else {
            xbox = new CommandXboxController(0);
            xbox2 = new CommandXboxController(1);
            pivotSubsystem = new PivotSubsystem(joystick);
            superstructure = new Superstructure(drivetrain, pivotSubsystem, elevSubsystem, xbox);

            configButtonBindings();
        }

        configDefaultCommands();
        configNamedCommands();
        drivetrain.configAutoBuilder();
        configSendableChoosers();

    }

    private void configDefaultCommands() {
        if (Utils.isSimulation()) {
            drivetrain.setDefaultCommand(drivetrain.applyRequest(
                    () -> drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed * polarityChooser.getSelected())
                            .withVelocityY(-joystick.getRawAxis(1) * MaxSpeed * polarityChooser.getSelected())
                            .withRotationalRate(-joystick2.getRawAxis(0) * MaxAngularRate * driverSpeedMultiplier)));
        } else {
            drivetrain.setDefaultCommand(drivetrain
                    .applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed * polarityChooser.getSelected())
                            .withVelocityY(-xbox.getLeftX() * MaxSpeed * polarityChooser.getSelected())
                            .withRotationalRate(-xbox.getRightX() * MaxAngularRate * driverSpeedMultiplier)));
        }
    }

    private void configSimButtonBindings() {
        // joystick.button(1).onTrue(pivotSubsystem.setAngle(-80));
        // joystick.button(2).onTrue(pivotSubsystem.setAngle(0));
        // joystick.button(3).onTrue(pivotSubsystem.setAngle(110));
        // joystick.button(4).whileTrue(pivotSubsystem.runIntake(1));

        //joystick.button(1).onTrue(superstructure.);
        joystick.button(2).onTrue(superstructure.scoreCoral());
        joystick.button(3).onTrue(superstructure.scoreCoralL4());
        //joystick.button(4).whileTrue(pivotSubsystem.runIntake(1));

        // joystick.button(1).onTrue(superstructure.setElevator(ELEV_DOWN));
        // joystick.button(2).onTrue(superstructure.scoreCoralL2());
        // joystick.button(3).onTrue(superstructure.scoreCoralL3());
        // joystick.button(4).onTrue(superstructure.scoreCoralL4());

        joystick2.button(1).onTrue(superstructure.setElevator(0.05));
        joystick2.button(2).onTrue(superstructure.setElevator(0.5));
        joystick2.button(3).onTrue(superstructure.setElevator(1.0));
        joystick2.button(4).onTrue(superstructure.setElevator(1.5));


    }

    private void configButtonBindings() {

        // Driver controls
        xbox.leftBumper().onTrue(superstructure.removeAlgaeLow().alongWith(drivetrain
        .applyRequest(() -> teleopDrive.withVelocityX(-xbox.getLeftY() * MaxSpeed * polarityChooser.getSelected() * 0.2)
                .withVelocityY(-xbox.getLeftX() * MaxSpeed * polarityChooser.getSelected() * 0.2)
                .withRotationalRate(xbox.getRightX() * MaxAngularRate * 0.5))));
        xbox.rightBumper().onTrue(superstructure.removeAlgaeHigh().alongWith(drivetrain
        .applyRequest(() -> teleopDrive.withVelocityX(-xbox.getLeftY() * MaxSpeed * polarityChooser.getSelected() * 0.2)
                .withVelocityY(xbox.getLeftX() * MaxSpeed * polarityChooser.getSelected()*0.2)
                .withRotationalRate(-xbox.getRightX() * MaxAngularRate * 0.5))));


        xbox.leftTrigger()
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withVelocityX(-xbox.getLeftY()
                                * polarityChooser.getSelected()
                                * 0.2 * MaxSpeed) // Drive
                        .withVelocityY(-xbox.getLeftX()
                                * polarityChooser.getSelected() *
                                0.2 * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                                -xbox.getRightX() * 0.7 * MaxAngularRate * driverSpeedMultiplier)));
        xbox.rightTrigger()
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withVelocityX(-xbox.getLeftY()
                                * polarityChooser.getSelected()
                                * 1.2 * MaxSpeed) // Drive
                        .withVelocityY(-xbox.getLeftX()
                                * polarityChooser.getSelected() *
                                1.2 * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                                -xbox.getRightX() * 3.2 * MaxAngularRate * driverSpeedMultiplier)));

        xbox.povRight().whileTrue(
                drivetrain.applyRequest(() -> teleopDrive.withVelocityX(0.0)
                        .withVelocityY(-0.6)
                        .withRotationalRate(0.0))
                        .until(() -> !drivetrain.seesRightSensor()));

        xbox.povLeft().whileTrue(
                drivetrain.applyRequest(() -> teleopDrive.withVelocityX(0.0)
                        .withVelocityY(0.6)
                        .withRotationalRate(0.0))
                        .until(() -> !drivetrain.seesLeftSensor()));

        xbox.povUp().whileTrue(
                drivetrain.applyRequest(() -> teleopDrive.withVelocityX(0.4)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)));

        xbox.povDown().whileTrue(
                drivetrain.applyRequest(() -> teleopDrive.withVelocityX(-0.4)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)));

        xbox.start().onTrue(resetGyro());
        xbox.back().onTrue(resetGyro180());

        xbox.a().onTrue(superstructure.storeAlgae());//
        xbox.b().onTrue(superstructure.scoreAlgae());
        xbox.x().onTrue(pivotSubsystem.setAngle(STOW_ANGLE));
        xbox.y().onTrue(pivotSubsystem.setAngle(-30));

        //xbox.x().onTrue(pivotSubsystem.setpoint = pivotSubsystem.setpoint - 2);
        //xbox.y().onTrue(pivotSubsystem.setpoint = pivotSubsystem.setpoint + 2);

        //operator bindings
        xbox2.povUp().onTrue(superstructure.scoreCoralL4());
        xbox2.povDown().onTrue(superstructure.getCoral());
        xbox2.povLeft().whileTrue(pivotSubsystem.runIntake(0.6));
        xbox2.povRight().onTrue(superstructure.scoreCoral());

        xbox2.x().onTrue(superstructure.setElevator(ELEV_L2));
        xbox2.b().onTrue(superstructure.setElevator(ELEV_L3));
        xbox2.a().onTrue(superstructure.setElevator(ELEV_DOWN));// TODO contemplate what to do about l1
        xbox2.y().onTrue(superstructure.setElevator(ELEV_L4));

        xbox2.leftBumper().onTrue(superstructure.placeAlgaeInBarge());
        xbox2.rightBumper().onTrue(superstructure.downFromBarge());
    }

    private void configNamedCommands() {
        NamedCommands.registerCommand("elevDown", superstructure.setElevator(ELEV_DOWN));
        NamedCommands.registerCommand("prepElevL4", superstructure.setElevator(ELEV_L4));
        NamedCommands.registerCommand("prepElevL3", superstructure.setElevator(ELEV_L3));
        NamedCommands.registerCommand("prepElevL2", superstructure.setElevator(ELEV_L2));
        NamedCommands.registerCommand("prepElevL1", superstructure.setElevator(ELEV_L1));
        NamedCommands.registerCommand("prepElevCoralIntake", superstructure.setElevator(ELEV_DOWN));
        NamedCommands.registerCommand("down", superstructure.down());
        NamedCommands.registerCommand("prepPivotL4", superstructure.setPivot(SCORE_CORAL_L4_ANGLE));
        NamedCommands.registerCommand("prepPivotReef", superstructure.setPivot(SCORE_CORAL_ANGLE));
        NamedCommands.registerCommand("prepPivotCoralIntake", superstructure.setPivot(INTAKE_CORAL_ANGLE));
        NamedCommands.registerCommand("prepPivotAlgae", superstructure.setPivot(INTAKE_ALGAE_ANGLE));

        NamedCommands.registerCommand("shoot", superstructure.outtakeCoral());
        NamedCommands.registerCommand("intake", superstructure.intakeCoral());

        NamedCommands.registerCommand("grabAlgaeHigh", superstructure.intakeAlgaeL3());
        NamedCommands.registerCommand("grabAlgaeLow", superstructure.intakeAlgaeL2());
        NamedCommands.registerCommand("holdAlgae", superstructure.holdAlgae());

        NamedCommands.registerCommand("autoAlignLeft", superstructure.autoAlignLeft());
        NamedCommands.registerCommand("autoAlignRight", superstructure.autoAlignRight());


        NamedCommands.registerCommand("autoScoreCoral", superstructure.autoScoreCoral());
        NamedCommands.registerCommand("autoScoreCoralL4", superstructure.autoScoreCoralL4());
        NamedCommands.registerCommand("Test", print("Auton test"));

        // TODO add more commands
    }

    private void configSendableChoosers() {
        autoChooser = drivetrain.autoChooser;
        SmartDashboard.putData("Auto Chooser", autoChooser);

        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);
        SmartDashboard.putData("Polarity", polarityChooser);

        driverChooser.setDefaultOption("None (autonomous craziness - DONT TRY)", Driver.Other);
        driverChooser.addOption("Karter", Driver.Karter);
        driverChooser.addOption("Cooper", Driver.Cooper);
        driverChooser.addOption("Walter", Driver.Walter);
        driverChooser.addOption("Caroline", Driver.Caroline);
        driverChooser.addOption("David", Driver.David);
        driverChooser.addOption("Riley", Driver.Riley);
        driverChooser.addOption("Jaxson", Driver.Jaxson);
        driverChooser.addOption("Laura", Driver.Laura);
        driverChooser.addOption("Cece", Driver.Cece);
        driverChooser.addOption("Abe Perryham", Driver.Abe_Perryham);
        driverChooser.addOption("Kennedy", Driver.Kennedy);
        driverChooser.addOption("Robert", Driver.Robert);
        driverChooser.addOption("Mentor", Driver.Mentor);

    }

    private Command resetGyro() {
        return Commands.runOnce(() -> {
            drivetrain.getPigeon2().reset();

        });
    }

    private Command resetGyro180() {
        return Commands.runOnce(() -> {
            drivetrain.getPigeon2().setYaw(180);
        });
    }

    /* NEEDS to be called periodically */
    public void updateDriverMultiplier() {
        if (driverChooser.getSelected() == Driver.Karter | driverChooser.getSelected() == Driver.Abe_Perryham) {
            driverSpeedMultiplier = 1.2;
        } else if (driverChooser.getSelected() == Driver.David) {
            driverSpeedMultiplier = 0.3;
        } else if (driverChooser.getSelected() == Driver.Robert | driverChooser.getSelected() == Driver.Kennedy |
                driverChooser.getSelected() == Driver.Jaxson | driverChooser.getSelected() == Driver.Cece) {
            driverSpeedMultiplier = 0.8;
        } else if (driverChooser.getSelected() == Driver.Other) {
            driverSpeedMultiplier = (Math.random()/2) + 0.7;
        } else {
            driverSpeedMultiplier = 1;
        }
    }

    /*NEEDS to be called periodically */
    public void preventTipping(){
        if(drivetrain.getPigeon2().getPitch().getValueAsDouble() > 35 | drivetrain.getPigeon2().getRoll().getValueAsDouble() > 35){
            elevSubsystem.setHeight(ELEV_DOWN);
        }
    }

}
