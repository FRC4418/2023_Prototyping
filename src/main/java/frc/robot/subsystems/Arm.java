// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

public class Arm extends SubsystemBase {
  /** Creates a new Claw. */

  // Initializes the motors
  final static WPI_TalonSRX motor1 = new WPI_TalonSRX(1);
  final static WPI_TalonSRX motor2 = new WPI_TalonSRX(2);


  public Arm() {
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    // Sets a motor as inverted
    motor1.setInverted(true);
  }

  public void spin(){
    motor1.set(0.75);
    motor2.set(0.75);
  }

  public void stop(){
    motor1.set(0.0);
    motor2.set(0.0);
  }

  @Override
  public void periodic() {

  }
}
