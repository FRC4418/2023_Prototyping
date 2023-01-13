// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
  

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  final WPI_TalonFX grabberMotor = new WPI_TalonFX(01);

  public GrabberSubsystem() {
    grabberMotor.configFactoryDefault();
  }

  public void spin(int speed){
    grabberMotor.set((double)speed);
  }

  public void stop(){
    grabberMotor.set(0.0);
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
