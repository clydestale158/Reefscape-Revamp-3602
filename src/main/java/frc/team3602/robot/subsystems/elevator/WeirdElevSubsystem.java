package frc.team3602.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.team3602.robot.Constants.HardwareConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WeirdElevSubsystem extends SubsystemBase{
    private final TalonFX leadMotor = new TalonFX(ELEV_LEAD_MOTOR_ID);
    private final TalonFX followerMotor = new TalonFX(ELEV_FOLLOW_MOTOR_ID);
    
    private TalonElevator elevator;

    public final double startingHeight = 0.6;

    public final ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60(2), ELEV_GEARING, 8, 0.5, -0.01, Units.inchesToMeters(72), true, startingHeight);

    public WeirdElevSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = ELEV_CURRENT_LIMIT;

        FeedbackConfigs feedbackCfg = cfg.Feedback;
        feedbackCfg.SensorToMechanismRatio = ELEV_GEARING;

        MotionMagicConfigs controllerCfg = cfg.MotionMagic;
        controllerCfg.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20)).withMotionMagicAcceleration(10)
        .withMotionMagicJerk(80);
        //TODO up with testing irl

        Slot0Configs slot0 = cfg.Slot0;

        if (Utils.isSimulation()) {
            slot0.kS = 0.0;
            slot0.kG = 0.05;//0.091;// PRE VOLTAGE MULTIPLICATION -> 0.69;//.7> && .67< 
            slot0.kA = 0.01;
            slot0.kV = 0.01;
            slot0.kP = 0.4;
            slot0.kI = 0.0;
            slot0.kD = 0.0;
        } else {
            slot0.kS = 0.0;
            slot0.kG = 1.0;
            slot0.kA = 0.2;
            slot0.kV = 0.1;
            slot0.kP = 0.0;
            slot0.kI = 0.0;
            slot0.kD = 0.0;
        }

        elevator = new TalonElevator("Elevator", leadMotor, followerMotor, false, 0, cfg, elevSim);
    }

    public Command setHeight(double newHeight){
        return runOnce(() ->{
            elevator.setHeight(newHeight);
        });
    }

    public boolean isNearGoal(){
        return MathUtil.isNear(elevator.setpoint, elevator.getEncoder(), 1.5);
    }

    @Override 
    public void simulationPeriodic(){
        elevator.updateSim();
    }

    @Override
    public void periodic(){
        elevator.updateDashboard();
        elevator.updateMotorControl();
    }
}
