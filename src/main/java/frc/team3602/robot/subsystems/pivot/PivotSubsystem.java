package frc.team3602.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.team3602.robot.Constants.HardwareConstants.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * Weird subsystem that works w the utility type class that I don't really like,
 * but we may end up using
 */
public class PivotSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private final TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    private final LaserCan laser = new LaserCan(INTAKE_LASER_ID);
    private CANcoder pivotEncoder;

    private CommandJoystick joystick;
    public final double startingAngle = 0;
    public double intakeSpeed;// ONLY USED FOR LOGGING AND SIM

    private ArmFeedforward ffeController;

    public final TalonPivot pivot;
    public final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), PIVOT_GEARING,
            SingleJointedArmSim.estimateMOI(0.2, 7), 0.2, Units.degreesToRadians(-120), Units.degreesToRadians(140),
            true, Units.degreesToRadians(startingAngle));

    public PivotSubsystem(CommandJoystick joystick) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = PIVOT_CURRENT_LIMIT;

        FeedbackConfigs feedbackCfg = cfg.Feedback;
        feedbackCfg.SensorToMechanismRatio = PIVOT_GEARING;

        MotionMagicConfigs controllerCfg = cfg.MotionMagic;
        controllerCfg.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)).withMotionMagicAcceleration(30)
                .withMotionMagicJerk(4);
        // TODO up with testing irl

        Slot0Configs slot0 = cfg.Slot0;

        if (Utils.isSimulation()) {
            this.joystick = joystick;

            slot0.kS = 0.0;
            // slot0.kG = 0.062;//.59<//PRE VOLTAGE MULTIPLICATION -> 0.235;//.24> && .23<
            slot0.kA = 0.03;
            slot0.kV = 0.03;
            slot0.kP = 0.02;//
            slot0.kI = 0.0;
            slot0.kD = 0.01;

            ffeController = new ArmFeedforward(0, 0.26, 0.2);//.27< .25<

        } else {
            slot0.kS = 0.0;
            slot0.kA = 0.06;
            slot0.kV = 0.04;
            slot0.kP = 0.05;
            slot0.kI = 0.0;
            slot0.kD = 0.0;

            ffeController = new ArmFeedforward(0, 0.27, 0);//.27

            pivotEncoder = new CANcoder(PIVOT_CANCODER_ID);

            // encoder configs
            var magnetSensorConfigs = new MagnetSensorConfigs();
            magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;
            pivotEncoder.getConfigurator().apply(magnetSensorConfigs);
        }

        pivot = new TalonPivot("Pivot", pivotMotor, pivotSim, cfg);
    }

    /** Run once command that changes the setpoint of the pivot */
    public Command setAngle(double newAngle) {
        return runOnce(() -> {
            pivot.setAngle(newAngle);
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

    public boolean isNearGoal() {
        return MathUtil.isNear(pivot.setpoint, pivot.getEncoder(), 5);
    }

    public boolean hasAlgae() {
        if (RobotBase.isSimulation()) {
            return joystick.button(2).getAsBoolean();
        } else {
            return intakeMotor.getTorqueCurrent().getValueAsDouble() > 20;// TODO change w testing
        }
    }

    public double getFfe() {
        return ffeController.calculate(Units.degreesToRadians(pivot.getEncoder()), pivotMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        pivot.updateSim();
    }

    @Override
    public void periodic() {
        pivot.updateDashboard(); // TODO take out if necessary - potential fix for periodic overruns
        pivot.updateMotorControl(getFfe());

        SmartDashboard.putBoolean("Intake sensor", sensorIsTriggered());
    }
}
