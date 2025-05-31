package frc.team3602.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.print;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.drive.generated.TunerConstants;
import frc.team3602.robot.subsystems.elevator.ElevSubsystem;
import frc.team3602.robot.subsystems.pivot.PivotSubsystem;
import frc.team3602.robot.vision.Vision;

public class RobotContainer {
    private CommandJoystick joystick;
    private CommandJoystick joystick2;
    private CommandXboxController xbox;
    private CommandXboxController xbox2;


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final DrivetrainSubsystem drivetrain = TunerConstants.createDrivetrain();
    private final ElevSubsystem elevSubsystem = new ElevSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem(joystick);

    private final Vision vision = new Vision();

    public SendableChooser<Command> autoChooser;
    private final SendableChooser<Double> polarityChooser = new SendableChooser<>();

    public RobotContainer(){
        

        //SmartDashboard.putData(autoChooser);
        SmartDashboard.putData("Polarity", polarityChooser);
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);

        if(Utils.isSimulation()){
            joystick = new CommandJoystick(0);
            joystick2 = new CommandJoystick(1);
            configSimButtonBindings();
        } else {
            xbox = new CommandXboxController(0);
            xbox2 = new CommandXboxController(1);

            configButtonBindings();
        }
        configDefaultCommands();
        configNamedCommands();
        drivetrain.configAutoBuilder(autoChooser);

    }

    private void configDefaultCommands(){
        if(Utils.isSimulation()){
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getRawAxis(0)*MaxSpeed * polarityChooser.getSelected()).withVelocityY(joystick.getRawAxis(1) * MaxSpeed * polarityChooser.getSelected()).withRotationalRate(-joystick2.getRawAxis(0)*MaxAngularRate)));
        } else {
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-xbox.getLeftY()*MaxSpeed * polarityChooser.getSelected()).withVelocityY(xbox.getLeftX() * MaxSpeed * polarityChooser.getSelected()).withRotationalRate(-xbox.getRightX()*MaxAngularRate)));
        }
    }

    private void configSimButtonBindings(){
        joystick.button(1).onTrue(pivotSubsystem.setAngle(-80));
        joystick.button(2).onTrue(pivotSubsystem.setAngle(0));
        joystick.button(3).onTrue(pivotSubsystem.setAngle(110));
        joystick.button(4).whileTrue(pivotSubsystem.runIntake(1));

        joystick2.button(1).onTrue(elevSubsystem.setHeight(0.05));
        joystick2.button(2).onTrue(elevSubsystem.setHeight(0.5));        
        joystick2.button(3).onTrue(elevSubsystem.setHeight(1.0));        
        joystick2.button(4).onTrue(elevSubsystem.setHeight(1.5));
    }

    private void configButtonBindings(){

    }

    private void configNamedCommands(){
        NamedCommands.registerCommand("Test", print("Auton test"));
    }



    public void updateVision(){

    }

}
