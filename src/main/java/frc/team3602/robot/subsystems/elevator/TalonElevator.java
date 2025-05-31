package frc.team3602.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonElevator {
    private final String elevName;

    public final TalonFX leadMotor;
    public final TalonFX followerMotor;

    public double setpoint = 0;

    public ElevatorSim elevSim;

    private TalonFXConfiguration configs;
    private final MotionMagicVoltage controller;

    public TalonElevator(String elevName, TalonFX leadMotor, TalonFX followerMotor, boolean followerOpposition,
            double startingHeight, TalonFXConfiguration configs, ElevatorSim elevSim) {
        this.elevName = elevName;
        this.leadMotor = leadMotor;
        this.followerMotor = followerMotor;

        this.elevSim = elevSim;

        controller = new MotionMagicVoltage(setpoint);

        this.configs = configs;
       
        StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            leaderStatus = leadMotor.getConfigurator().apply(configs);
            if (leaderStatus.isOK())
                break;
        }
        if (!leaderStatus.isOK()) {
            System.out.println("Could not configure Elevator Lead Motor. Error: " + leaderStatus.toString());
        }

        StatusCode followerStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            followerStatus = followerMotor.getConfigurator().apply(configs);
            if (followerStatus.isOK())
                break;
        }

        if (!followerStatus.isOK()) {
            System.out.println("Could not configure Elevator Follower Motor. Error: " + followerStatus.toString());
        }

        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), followerOpposition));
    }

    /**method that returns the elevator rotor position (or meters in a simulation) */
    public double getEncoder() {
        if (Utils.isSimulation()) {
            return elevSim.getPositionMeters();
        } else {
            return leadMotor.getRotorPosition().getValueAsDouble();
        }
    }

    /**
     * call to set motors voltages to the parameter. Useful for typical PID/ffe or
     * to stop the motors
     */
    public void setMotorVoltage(double volts) {
        leadMotor.setVoltage(volts);
    }

    /** changes the setpoint that is used with the updateMotorControl method */
    public void setHeight(double newHeight) {
        setpoint = newHeight;
    }

    /**
     * call (referably periodically) to set the control of the motors using motion
     * magic
     */
    public void updateMotorControl() {
        leadMotor.setControl(controller.withPosition(setpoint - getEncoder()).withSlot(0));
    }

    /**Updates the elevator sim input */
    public void updateSim() {
        elevSim.setInput(leadMotor.getMotorVoltage().getValueAsDouble());
        elevSim.update(0.001);
    }

    /**Puts some helpful numbers to Smartdashboard */
    public void updateDashboard() {
        SmartDashboard.putNumber(elevName + " encoder", getEncoder());
        SmartDashboard.putNumber(elevName + " setpoint", setpoint);

        SmartDashboard.putNumber(elevName + " set voltage", leadMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(elevName + " follower set voltage",
                followerMotor.getMotorVoltage().getValueAsDouble());
    }
}
