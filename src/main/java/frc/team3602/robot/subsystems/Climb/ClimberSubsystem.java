/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems.Climb;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.team3602.robot.Constants.HardwareConstants.CLIMB_CURRENT_LIMIT;
import static frc.team3602.robot.Constants.HardwareConstants.CLIMB_GEARING;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.Constants.ClimberConstants;
import frc.team3602.robot.Constants.PivotConstants;
import frc.team3602.robot.subsystems.pivot.TalonPivot;

// TODO: Write Simulation?
public class ClimberSubsystem extends SubsystemBase {

    // Motor ID
    private final TalonFX motor = new TalonFX(ClimberConstants.motorCANId);

    // PID controllers
    private PIDController climbController;
    private ArmFeedforward climbFeedforward;

    // Set Point for Climber
    public double setClimbAngle = 0.0;

    // // Variables
    public final static double INTAKE_ANGLE = 110;// TODO check/fix irl
    public final static double STOW_ANGLE = 80;// TODO check/fix irl
    public final static double INTAKE_ALGAE_ANGLE = 110;// TODO check/fix irl

    public final static double SCORE_ANGLE = 80;// TODO check/fix irl
    public final static double SCORE_L4_ANGLE = 70;// TODO check/fix irl

    // // Encoder
    // private double climberEncoder = 0.0;

    // Configs
    private TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
    private ArmFeedforward ffeController;

    public ClimberSubsystem(CommandJoystick joystick2) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
    }

    // Climb sim
    public TalonPivot pivot;
    public final SingleJointedArmSim climbSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), CLIMB_GEARING,
            SingleJointedArmSim.estimateMOI(1, 1), 1, Units.degreesToRadians(1), Units.degreesToRadians(2), true,
            Units.degreesToRadians(3));

    // Costructor
    public ClimberSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs outputCfg = cfg.MotorOutput;
        outputCfg.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs limitCfg = cfg.CurrentLimits;
        limitCfg.StatorCurrentLimit = CLIMB_CURRENT_LIMIT;

        if (Utils.isSimulation()) {
            // climbMotorConfig = new TalonFXConfiguration();
            climbController = new PIDController(0, 0, 0);
            climbFeedforward = new ArmFeedforward(0, 0, 0, 0);
        } else {
            climbController = new PIDController(0, 0, 0);
            climbFeedforward = new ArmFeedforward(0, 0, 0, 0);
        }
    }

    // Simulation
    // public final SingleJointedArmSim climbSim = new
    // SingleJointedArmSim(DCMotor.getKrakenX60(1),
    // ClimberConstants.gearing,
    // SingleJointedArmSim.estimateMOI(ClimberConstants.lengthMeters,
    // ClimberConstants.massKG),
    // ClimberConstants.lengthMeters, -12, 1, true, 0);
    // // TODO: Find approriate travel distance in Rads, and learn what -12 means.
    // public double climberVislength;
    // public Mechanism2d climberSimMech = new Mechanism2d(1, 1);
    // public MechanismRoot2d climberRoot;
    // // public final MechanismLigament2d climberVis;
    // public MechanismLigament2d climberVis;

    // // Sim Init
    // public ClimberSubsystem(MechanismRoot2d climberRoot, double climberVislength)
    // {
    // motor.setPosition(0.0);
    // this.climberRoot = climberRoot;
    // this.climberVis = this.climberRoot
    // .append(new MechanismLigament2d("Climber Ligament", 1, 0, 1, new
    // Color8Bit(Color.kIndianRed)));
    // this.climberVislength = climberVislength;

    // configClimbSubsys();

    // }

    // Commands For Reference
    public Command setAngle(double newAngle) {
        return runOnce(() -> {
            pivot.setAngle(newAngle);
        });
    }

    public double getEncoder() {
        if (Utils.isSimulation()) {
            return climbSim.getAngleRads();
        } else {
            return motor.getRotorPosition().getValueAsDouble();
        }
    }

    public boolean isNearGoal() {
        return MathUtil.isNear(pivot.setpoint, pivot.getEncoder(), 1);
    }

    public double getFfe() {
        return ffeController.calculate(pivot.getEncoder(), motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", motor.getPosition().getValueAsDouble());

        if (Utils.isSimulation()) {
            pivot.updateSim();
        } else {

        }
    }
}