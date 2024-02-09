package frc.robot.subsystems;

 import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TalonVelocityIO;

import org.littletonrobotics.junction.Logger;

/**
 * <h3>ShooterSubsystem</h3>
 * This subsystem controls the shooter
 */
public class ShooterSubsystem extends SubsystemBase{
    private TalonVelocityIO IO_Left;
    private TalonVelocityIO IO_Right;

    public ShooterSubsystem(TalonVelocityIO LeftIO, TalonVelocityIO RightIO) { 
        IO_Left = LeftIO;
        IO_Right = RightIO;
        IO_Left.getTalon().setInverted(true);
        IO_Right.getTalon().setInverted(true);
    }
    
    /**
    * <h3>setSpeed</h3>
    * @param leftSpeed the speed the left wheel will be set to
    * @param rightSpeed the speed the right wheel will be set to
    */
    public void setSpeed(double leftSpeed, double rightSpeed, double leftAccel, double rightAccel) {
        IO_Left.setSpeed(leftSpeed, leftAccel);
        IO_Right.setSpeed(rightSpeed, rightAccel);
    }

    /**
    * <h3>setSpeed</h3>
    * @param leftSpeed the speed the left wheel will be set to
    * @param rightSpeed the speed the right wheel will be set to
    */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        IO_Left.setSpeed(leftSpeed);
        IO_Right.setSpeed(rightSpeed);
    }

    /**
    * <h3>getLeftMotorSpeed</h3>
    * @return The current motor speed of the left wheel in rps
    */
    public double getLeftMotorSpeed() {
        return IO_Left.getSpeed();
    }

    /**
    * <h3>getRightMotorSpeed</h3>
    * @return The current motor speed of the right wheel in rps
    */
    public double getRightMotorSpeed() {
        return IO_Right.getSpeed();
    }

    /**
    * <h3>getLeftVoltage</h3>
    * @return The current voltage of the left motor
    */
    public double getLeftVoltage() {
        return IO_Left.getVoltage();
    }

    /**
    * <h3>getRightVoltage</h3>
    * @return The current voltage of the right motor
    */
    public double getRightVoltage() {
        return IO_Right.getVoltage();
    }

    /**
    * <h3>getLeftTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    public double getLeftTargetVelocity() {
        return IO_Left.getTargetVelocity();
    }

    /**
    * <h3>getRightTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    public double getRightTargetVelocity() {
        return IO_Right.getTargetVelocity();
    }

    /**
    * <h3>stop</h3>
    * This sets the shooter's speed to 0
    */
    public void stop() {
        setSpeed(0,0,0,0);
    }

    @Override
    public void periodic() {
        IO_Left.runSim();
        IO_Right.runSim();
        
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/Velocity" ,getLeftMotorSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/Voltage" ,getLeftVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/SetPoint" ,getLeftTargetVelocity());

        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/Velocity" ,getRightMotorSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/Voltage" ,getRightVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/SetPoint" ,getRightTargetVelocity());
    }
}

