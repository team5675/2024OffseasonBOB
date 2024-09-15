// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.RunMotor;
import frc.robot.commands.StopMotor;
import frc.robot.subsystems.Motor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  CommandXboxController xboxController; 
  public Motor motor;

  // DigitalInput proxSensor;
  //   Trigger isTripped;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    motor = new Motor();
    xboxController = new CommandXboxController(0);
    
    // proxSensor = new DigitalInput(4);
    // isTripped = new Trigger(proxSensor::get);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {

  //xboxController.a().whileTrue(Commands.run(()->motor.useEncoder()));
  

   xboxController.b().whileTrue(new RunMotor(motor)).onFalse(new StopMotor(motor));

   //motor.isTripped.onFalse(Commands.runOnce(()->motor.setMotorSpeed(0),motor));

   //xboxController.b().whileTrue(motor.runMotorWhenTrippedCommand());

   xboxController.leftTrigger().onTrue(Commands.run(()->motor.setmotorRevolutions(10),motor))
   .onFalse(Commands.run(()->motor.stopMotor(), motor));

    //With Joystick
   xboxController.x().onTrue(Commands.run(()->motor.setMotorSpeed(xboxController.getLeftY()),motor)
   .alongWith(Commands.run(()-> System.out.println(xboxController.getLeftY()))))
   .onFalse(Commands.run(()->motor.stopMotor(), motor));
  
    xboxController.rightTrigger().onTrue(Commands.run(()->motor.setWristAngle(),motor)).onFalse(
      Commands.run(()->motor.setMotorSpeed(0), motor)
    );

   //xboxController.a().onTrue(Commands.run(()->motor.motor.set(0), motor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
}
