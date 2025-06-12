package frc.team3602.robot.subsystems.pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Weird utility type class that I dont really like but we might end up using */
public class TalonPivot {
    private final String pivotName;
    private final TalonFX motor;

    public double setpoint;

    private final SingleJointedArmSim pivotSim;

    private final TalonFXConfiguration configs;
    private final MotionMagicVoltage controller;

    public TalonPivot(String name, TalonFX motor, SingleJointedArmSim pivotSim, TalonFXConfiguration configs){
        this.pivotName = name;
        this.motor = motor;
        this.pivotSim = pivotSim;
        this.configs = configs;

        controller = new MotionMagicVoltage(setpoint);

        StatusCode pivotStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            pivotStatus = motor.getConfigurator().apply(configs);
            if (pivotStatus.isOK())
                break;
        }
        if (!pivotStatus.isOK()) {
            System.out.println("Could not configure Pivot Motor. Error: " + pivotStatus.toString());
        }
    }

     /**method that returns the motor rotor position (or degrees of the sim in a simulation) */
    public double getEncoder() {
        if (Utils.isSimulation()) {
            return Units.radiansToDegrees(pivotSim.getAngleRads());
        } else {
            return motor.getRotorPosition().getValueAsDouble();
        }
    }

    /**
     * call to set motors voltages to the parameter. Useful for typical PID/ffe or
     * to stop the motors
     */
    public void setMotorVoltage(double volts) {
        motor.setVoltage(volts);
    }

    /** changes the setpoint that is used with the updateMotorControl method */
    public void setAngle(double newAngle) {
        setpoint = newAngle;
    }


    /**
     * call (preferably periodically) to set the control of the motors using motion
     * magic
     */
    public void updateMotorControl(double ffe) {
        motor.setControl(controller.withPosition(setpoint - getEncoder()).withSlot(0).withFeedForward(ffe));
    }

    /**Updates the elevator sim input */
    public void updateSim() {
        pivotSim.setInput(motor.getMotorVoltage().getValueAsDouble() * 4);
        pivotSim.update(0.001);
    }

    /**Puts some helpful numbers to Smartdashboard */
    public void updateDashboard() {
        SmartDashboard.putNumber(pivotName + " encoder", getEncoder());
        SmartDashboard.putNumber(pivotName + " setpoint", setpoint);

        SmartDashboard.putNumber(pivotName + " set voltage", motor.getMotorVoltage().getValueAsDouble());
    }
}
