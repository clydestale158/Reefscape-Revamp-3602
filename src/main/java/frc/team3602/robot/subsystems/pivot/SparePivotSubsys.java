package frc.team3602.robot.subsystems.pivot;

import static frc.team3602.robot.Constants.HardwareConstants.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.simulation.SingleJointedArmSim.estimateMOI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class SparePivotSubsys extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private final TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    private CANcoder pivotEncoder;

    private LaserCan laser;
    private CommandJoystick joystick;

    private PIDController controller;
    private ArmFeedforward ffeController;

    public final double startingAngle = 30;
    private double setpoint;
    public double intakeSpeed;

    public final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), PIVOT_GEARING,
            estimateMOI(0.3, 1), 0.3, -3, 3, true, startingAngle);

    /** Simulation Constructor */
    public SparePivotSubsys(CommandJoystick joystick) {
        if (RobotBase.isSimulation()) {
            this.joystick = joystick;

            controller = new PIDController(0, 0, 0);
            ffeController = new ArmFeedforward(0, 0.3, 0, 0);
        } else {// but in case we are monkeys who use this constructor irl, we include this
            pivotEncoder = new CANcoder(PIVOT_CANCODER_ID);

            // encoder configs
            var magnetSensorConfigs = new MagnetSensorConfigs();
            magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;
            pivotEncoder.getConfigurator().apply(magnetSensorConfigs);

            controller = new PIDController(0, 0, 0);
            ffeController = new ArmFeedforward(0, 0.3, 0, 0);
        }

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = PIVOT_CURRENT_LIMIT;
    }

    /** IRL Constructor, NOT for sim */
    public SparePivotSubsys() {
        // no sim stuff here

        pivotEncoder = new CANcoder(PIVOT_CANCODER_ID);

        // encoder configs
        var magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        pivotEncoder.getConfigurator().apply(magnetSensorConfigs);

        controller = new PIDController(0, 0, 0);
        ffeController = new ArmFeedforward(0, 0.3, 0, 0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = PIVOT_CURRENT_LIMIT;
    }

    /** Run once command that changes the setpoint of the pivot */
    public Command setAngle(double newAngle) {
        return runOnce(() -> {
            setpoint = newAngle;
        });
    }

    /**
     * Run end command that sets the speed of the intake motor, then sets it to 0
     * upon ending
     */
    public Command runIntake(double speed) {
        return runEnd(() -> {
            intakeMotor.set(speed);
            intakeSpeed = speed;
        }, () -> {
            intakeMotor.set(0);
            intakeSpeed = 0;
        });
    }

    /** Run once command that sets the speed of the intake motor */
    public Command setIntake(double speed) {
        return runOnce(() -> {
            intakeMotor.set(speed);
            intakeSpeed = speed;
        });
    }

    /**run end command that sets the intake motor and upon ending, sets it to a slow holding speed */
    public Command intakeAlgae() {
        return runEnd(() -> {
            setIntake(INTAKE_ALGAE_SPEED);
            intakeSpeed = INTAKE_ALGAE_SPEED;
        }, () -> {
            setIntake(HOLD_ALGAE_SPEED);
            intakeSpeed = HOLD_ALGAE_SPEED;
        });
    }

    public boolean sensorIsTriggered() {
        if (Utils.isSimulation()) {
            return joystick.button(1).getAsBoolean();
        } else {
            LaserCan.Measurement meas = laser.getMeasurement();
            return meas.distance_mm < 50;
        }
    }

    /**
     * method that returns the motor rotor position (or degrees of the sim in a
     * simulation)
     */
    public double getEncoder() {
        if (Utils.isSimulation()) {
            return Units.radiansToDegrees(pivotSim.getAngleRads());
        } else {
            return pivotMotor.getRotorPosition().getValueAsDouble();
        }
    }

    private double getEffort() {
        return ffeController.calculate(Units.degreesToRadians(getEncoder()),
                pivotMotor.getVelocity().getValueAsDouble()) + controller.calculate(getEncoder(), setpoint);
    }

    public boolean isNearGoal() {
        return MathUtil.isNear(setpoint, getEncoder(), 5);
    }

    public boolean hasAlgae() {
        if (RobotBase.isSimulation()) {
            return joystick.button(2).getAsBoolean();
        } else {
            return intakeMotor.getTorqueCurrent().getValueAsDouble() > 20;// TODO change w testing
        }
    }

    @Override
    public void simulationPeriodic() {
        pivotSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble() * 4);
        pivotSim.update(0.001);
    }

    @Override
    public void periodic() {
        pivotMotor.setVoltage(getEffort());
        SmartDashboard.putNumber("Pivot encoder", getEncoder());
        SmartDashboard.putNumber("Pivot setpoint", setpoint);

        SmartDashboard.putNumber("Pivot calculated effort", getEffort());
        SmartDashboard.putNumber("Pivot voltage", pivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake set speed", intakeSpeed);
        SmartDashboard.putBoolean("Intake sensor", sensorIsTriggered());
    }
}
