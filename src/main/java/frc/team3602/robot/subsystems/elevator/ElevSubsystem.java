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

/** just an elevator subsystem that does NOT use motion magic. **USE THIS** */
public class ElevSubsystem extends SubsystemBase {
    //Motors
    private final TalonFX motor = new TalonFX(ELEV_LEAD_MOTOR_ID);
    private final TalonFX followerMotor = new TalonFX(ELEV_FOLLOW_MOTOR_ID);

    //Controllers
    private PIDController controller;
    private ElevatorFeedforward ffeController;

    //Setpoints
    public final double startingHeight = 0;//TODO IRL - change back to 0
    private double setpoint = startingHeight;

    //Elev sim
    public final ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60(2), ELEV_GEARING, 3,
            Units.inchesToMeters(1), -0.1, 3, true, startingHeight);

    /**Constructor */
    public ElevSubsystem() {
        if (RobotBase.isSimulation()) {
            controller = new PIDController(1.0, 0, 0);//todo tune eventually
            ffeController = new ElevatorFeedforward(0, 0.0001, 0);
        } else {
            controller = new PIDController(0.09, 0, 0.0001);//$kp = .5 //TODO finish tuning. Was tested before tensioning the elevator, and was janky
            ffeController = new ElevatorFeedforward(0.8, 0.27, 0.9, 0.1);
        }

        //configurations
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;
        outputCfg.Inverted = InvertedValue.CounterClockwise_Positive;//TODO maybe change??

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = ELEV_CURRENT_LIMIT;

        //config application
        motor.getConfigurator().apply(cfg);
        followerMotor.getConfigurator().apply(cfg);
        
        //setting the control of the follower motor to be a follower of our lead motor
        followerMotor.setControl(new Follower(ELEV_LEAD_MOTOR_ID, false));
    }

    /**Run once command that changes the setpoint of the elevator */
    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            setpoint = newHeight;
        });
    }

    /**
     * method that returns the elevator rotor position (or meters in a simulation)
     */
    public double getEncoder() {
        if (Utils.isSimulation()) {
            return elevSim.getPositionMeters();
        } else {
            return motor.getRotorPosition().getValueAsDouble();
        }
    }

    /**returns the combined calculated effort of our ffe and pid controllers*/
    private double getEffort(){
        return ffeController.calculate(0)//motor.getVelocity().getValueAsDouble(), motor.getAcceleration().getValueAsDouble())// + //TODO debate velocity and acceleration things. it is new therefore scary (Usually it is 0)
        + controller.calculate(getEncoder(), setpoint);
    }

    /**returns true if the elev encoder and setpoint are within 1.5 units of each other */
    public boolean isNearGoal() {
        return MathUtil.isNear(setpoint, getEncoder(), 1.5);
    }

    @Override
    public void simulationPeriodic() {
        //update the elev sim
        elevSim.setInput(motor.getMotorVoltage().getValueAsDouble() * 28);
        elevSim.update(0.001);
    }

    @Override
    public void periodic() {
        //updating dashboard
        SmartDashboard.putNumber("Elevator encoder", getEncoder());
        SmartDashboard.putNumber("Elevator setpoint", setpoint);
        SmartDashboard.putNumber("Elevator velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator set voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator acceleration", motor.getAcceleration().getValueAsDouble());

        SmartDashboard.putNumber("Elevator follower set voltage",
                followerMotor.getMotorVoltage().getValueAsDouble());

        //updating motor voltage
        motor.setVoltage(getEffort());
    }
}
