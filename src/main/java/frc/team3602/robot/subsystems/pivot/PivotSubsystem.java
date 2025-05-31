package frc.team3602.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.team3602.robot.Constants.HardareConstants.*;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private final TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    private final LaserCan laser = new LaserCan(INTAKE_LASER_ID);
    private CANcoder pivotEncoder;

    private CommandJoystick joystick;
    public final double startingAngle = 0;
    public double intakeSpeed;// ONLY USED FOR LOGGING AND SIM

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
        controllerCfg.withMotionMagicCruiseVelocity(RotationsPerSecond.of(2)).withMotionMagicAcceleration(4);
        // TODO up with testing irl

        Slot0Configs slot0 = cfg.Slot0;

        if (Utils.isSimulation()) {
            this.joystick = joystick;

            slot0.kS = 0.0;
            slot0.kG = 1.0;
            slot0.kA = 0.2;
            slot0.kV = 0.1;
            slot0.kP = 0.0;
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

    public boolean sensorIsTriggered() {
        if (Utils.isSimulation()) {
            return joystick.button(1).getAsBoolean();
        } else {
            LaserCan.Measurement meas = laser.getMeasurement();
            return meas.distance_mm < 50;
        }
    }

    public boolean isNearGoal(){
        return MathUtil.isNear(pivot.setpoint, pivot.getEncoder(), 5);
    }

    @Override
    public void simulationPeriodic() {
        pivot.updateSim();
    }

    @Override
    public void periodic() {
        pivot.updateDashboard();
        pivot.updateMotorControl();

        SmartDashboard.putBoolean("Intake sensor", sensorIsTriggered());
    }
}
