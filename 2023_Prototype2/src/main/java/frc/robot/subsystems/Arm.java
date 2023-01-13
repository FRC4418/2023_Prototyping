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
  final static WPI_TalonSRX leftMotor = new WPI_TalonSRX(Ports.ArmPorts.leftClaw);


  public Arm() {
    leftMotor.configFactoryDefault();


    // Sets a motor as inverted
    leftMotor.setInverted(true);
  }

  public void closeClaw(Number percent) {
    leftMotor.set(ControlMode.PercentOutput, (double)percent);
  }

  @Override
  public void periodic() {

  }
}
