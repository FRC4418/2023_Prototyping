// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Claw extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(1);
  public Claw() {
    leftFrontMotor.configFactoryDefault();
  }

  public static void spin(Number speed){
    leftFrontMotor.set((double) speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
