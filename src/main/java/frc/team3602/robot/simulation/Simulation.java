package frc.team3602.robot.simulation;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.subsystems.elevator.ElevSubsystem;
import frc.team3602.robot.subsystems.pivot.PivotSubsystem;

public class Simulation extends SubsystemBase {
    private ElevSubsystem elevSubsys;
    private PivotSubsystem pivotSubsys;

    // Colors
    private final Color8Bit red = new Color8Bit(Color.kFirstRed);
    private final Color8Bit green = new Color8Bit(Color.kGreen);
    private final Color8Bit blue = new Color8Bit(Color.kSkyBlue);
    private final Color8Bit orange = new Color8Bit(Color.kPapayaWhip);

    private final Mech elevMech = new Mech("Elevator", 1.5, 1.5);
    private final Mech pivotMech = new Mech("Pivot", 1.5, 1.5);

    private final FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1),0.003, 5), DCMotor.getKrakenX60(1));

    public Simulation(ElevSubsystem elevSubsys, PivotSubsystem pivotSubsys){
        this.elevSubsys = elevSubsys;
        this.pivotSubsys = pivotSubsys;

        elevMech.addViz("Carriage", 0.75, 0.1, elevSubsys.startingHeight, 90, 70.0, orange);

        pivotMech.addViz("Pivot", 0.75, 0.75, 0.56, pivotSubsys.startingAngle, 10, orange);
        pivotMech.addViz2("Wheel", 1.5, 1.5, 0.07, 0, 10, blue);

        pivotMech.addViz3("Sensor", 0, 0, 1.5, 0, 30, red);

        SmartDashboard.putData("Elev Mech", elevMech.get());
        SmartDashboard.putData("Pivot Mech", pivotMech.get());
    }

    @Override
    public void simulationPeriodic(){
        intakeSim.setInput(pivotSubsys.intakeSpeed * 12);
        intakeSim.update(0.001);

        elevMech.viz.setLength(elevSubsys.elevSim.getPositionMeters());

        pivotMech.viz.setAngle(pivotSubsys.pivot.getEncoder(), new Rotation3d(0,pivotSubsys.pivot.getEncoder(), 0));
        
        pivotMech.viz2.setAngle(pivotMech.viz2.getAngle() + (intakeSim.getAngularVelocityRPM() * 0.02), new Rotation3d(0, pivotMech.viz2.getAngle(), 0));
        pivotMech.viz2.setRoot((Math.cos(Units.degreesToRadians(pivotMech.viz.getAngle())) * 0.56) + 0.75,
        (Math.sin(Units.degreesToRadians(pivotMech.viz.getAngle())) * 0.56) + 0.75);

        elevMech.viz.updatePublisher();
        pivotMech.viz.updatePublisher();
        pivotMech.viz2.updatePublisher();
    } 
}