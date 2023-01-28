// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ArmsOpen;
import frc.robot.commands.IntakePull;
import frc.robot.commands.IntakePush;
import frc.robot.commands.Testing;
import frc.robot.commands.dopeSlopeControl;
import frc.robot.constants.Ports;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DopeSlope;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  // The robot's subsystems and commands are defined here...
  private final Arms m_arms = new Arms();
  private final Intake m_intake = new Intake();
  private final ArmsOpen m_autoCommand = new ArmsOpen(m_arms);
  private final IntakePull m_autocommand = new IntakePull(m_intake);
  private final DopeSlope dopeslope = new DopeSlope();
  public final AutoGamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    /* Auto Builder */
  // Using the PathPlanner RamseteAutoBuilder constructor to create an AutoBuilder
  RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    m_robotDrive::getPose,
    m_robotDrive::resetOdometry,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    m_robotDrive.kinematics,
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
    m_robotDrive::getWheelSpeeds,
    new PIDConstants(DriveConstants.kPDriveVel, 0.0, 0.0),
    m_robotDrive::tankDriveVolts,
    SystemConstants.eventMap,
    m_robotDrive
    );
  
  /* Auto Commands
      Using the autoBuilder object constructed above, we can create an entire auto routine with the following:
        - Name of the command
        - Path group for that specific auto
          - This is defined in the AutoPaths.java file
      
  */
  private final Command m_auto1 = autoBuilder.fullAuto(AutoPaths.pathGroupAuto1); 

  // A chooser for autonomous commands
  SendableChooser<Command> m_autonomouschooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
    Shuffleboard.getTab("Auto Chooser").add(m_autonomouschooser);
    m_autonomouschooser.setDefaultOption("Auto 1", m_auto1);

    setEventMap();
  }

  public void setEventMap() {
    SystemConstants.eventMap.put("Spin motors",new InstantCommand(m_intake::intakeExtend, m_intake));
  }

  private void configureDefaultCommands() {
    dopeslope.setDefaultCommand(new dopeSlopeControl(dopeslope, driver));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.getLeftButton().whileTrue(new ArmsOpen(m_arms));
    driver.getTopButton().whileTrue(new IntakePull(m_intake));
    driver.getBottomButton().whileTrue(new IntakePush(m_intake));
  }

  // https://docs.google.com/drawings/d/1e4qhpc7L0whN3PPnOP-MKl21IsEmKWJkSbZKH311heM/edit?usp=sharing
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
