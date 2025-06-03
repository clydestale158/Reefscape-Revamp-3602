package frc.team3602.robot.subsystems.elevator;

import static frc.team3602.robot.Constants.HardwareConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**just an elevator subsystem that does NOT use motion magic */
public class SpareElevSubsys extends SubsystemBase{
    private final TalonFX motor = new TalonFX(ELEV_LEAD_MOTOR_ID);
    private final TalonFX followerMotor = new TalonFX(ELEV_FOLLOW_MOTOR_ID);

    private PIDController controller;
    private ElevatorFeedforward ffeController;

    private final double startingHeight = 0;
    private double setpoint = startingHeight;

    public final ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60(2), ELEV_GEARING, 4,
       Units.inchesToMeters(1), -0.1, 3, true, startingHeight);

    public SpareElevSubsys(){
        if(RobotBase.isSimulation()){
            controller = new PIDController(0, 0, 0);
            ffeController = new ElevatorFeedforward(0, 0.5, 0, 0);
        } else {
            controller = new PIDController(0, 0, 0);
            ffeController = new ElevatorFeedforward(0, 0.5, 0, 0); 

        }

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;
        //default is counterclockwise
        outputCfg.Inverted = InvertedValue.Clockwise_Positive;//TODO check

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = ELEV_CURRENT_LIMIT;

        followerMotor.setControl(new Follower(ELEV_LEAD_MOTOR_ID, false));
    }

     public Command setHeight(double newHeight){
        return runOnce(() ->{
            setpoint = newHeight;
        });
    }

/**method that returns the elevator rotor position (or meters in a simulation) */
public double getEncoder() {
    if (Utils.isSimulation()) {
        return elevSim.getPositionMeters();
    } else {
        return motor.getRotorPosition().getValueAsDouble();
    }
}

    public boolean isNearGoal(){
        return MathUtil.isNear(setpoint, getEncoder(), 1.5);
    }

    @Override 
    public void simulationPeriodic(){
        elevSim.setInput(motor.getMotorVoltage().getValueAsDouble() * 16);
        elevSim.update(0.001);    }


    @Override 
    public void periodic(){
SmartDashboard.putNumber("Elevator encoder", getEncoder());
        SmartDashboard.putNumber("Elevator setpoint", setpoint);

        SmartDashboard.putNumber("Elevator set voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator follower set voltage",
                followerMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Elevator velocity", motor.getVelocity().getValueAsDouble());
    }
}
