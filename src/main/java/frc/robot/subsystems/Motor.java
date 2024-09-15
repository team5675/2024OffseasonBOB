// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Add your docs here. */
public class Motor extends SubsystemBase {
    public CANSparkMax motor;
    public RelativeEncoder encoder;
    public SparkPIDController motorPID;

    public DigitalInput proxSensor;
    public Trigger isTripped;
    public final AbsoluteEncoder wristThroughBore;
    public final RelativeEncoder wristEncoder;
    
    public Motor(){
       motor = new CANSparkMax(Constants.MotorConstants.motorID, MotorType.kBrushless);
       //encoder = motor.getAlternateEncoder(42);
       motorPID = motor.getPIDController();
       wristThroughBore = motor.getAbsoluteEncoder();
       wristEncoder = motor.getEncoder();
       wristThroughBore.setPositionConversionFactor(360);
       wristThroughBore.setVelocityConversionFactor(360);
       motor.enableVoltageCompensation(12);
       motor.setSmartCurrentLimit(40);
       motor.setIdleMode(IdleMode.kBrake);
       
       double kMaxOutput = 1; 
        double kMinOutput = -1;
       motorPID.setP(0.02);
        motorPID.setI(0);
        motorPID.setD(0);
        motorPID.setOutputRange(kMinOutput, kMaxOutput);
        
       proxSensor = new DigitalInput(Constants.MotorConstants.proxSensorPort);
       isTripped = new Trigger(proxSensor::get);
       motor.burnFlash();
    }
    public void setMotorSpeed(double speed){
        motor.set(speed);
    }
    public void stopMotor(){
        motor.set(0);
    }
    public void setmotorRevolutions(double revolutions){
        motorPID.setFeedbackDevice(wristEncoder);
        motorPID.setReference(revolutions, CANSparkMax.ControlType.kPosition);
    }
    public void setWristAngle(){
        motorPID.setFeedbackDevice(wristThroughBore);
        motorPID.setReference(100,ControlType.kPosition);
    }
    public double getWristAngle(){
       return  wristThroughBore.getPosition();
    }
    public double getWristEncoderCounts(){
        return wristEncoder.getPosition();
    }
    public Command runMotorCommand(double speeds){
        return this.startEnd(()->motor.set(speeds), ()->motor.set(0));
    }
    public Command motorRevolutionsCommand(){
        return this.run(()->motorPID.setReference(1, ControlType.kPosition));
    }
    public Command runMotorWhenTrippedCommand(){
        if(isTripped.getAsBoolean()){
            return this.run(()->motor.set(Constants.MotorConstants.motorSpeed));
        }else{
            return this.run(()->motor.set(0));
        }
        //return Commands.either(this.runOnce(()->motor.set(0.2)), this.runOnce(()->motor.set(0)), isTripped);
    } 
    public void periodic(){
        SmartDashboard.putNumber("WristAngle", getWristAngle());
        SmartDashboard.putNumber("WristCounts", getWristEncoderCounts());
    }
}
