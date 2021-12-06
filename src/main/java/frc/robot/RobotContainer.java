// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrivebase m_swerveDrivebase = new SwerveDrivebase();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  public Joystick m_leftJoy = new Joystick(0);
  public Joystick m_rightJoy = new Joystick(1);


  private JoystickButton m_leftButton1 = new JoystickButton(m_leftJoy, 1);
  private JoystickButton m_leftButton2 = new JoystickButton(m_leftJoy, 2);
  private JoystickButton m_leftButton3 = new JoystickButton(m_leftJoy, 3);
  private JoystickButton m_leftButton4 = new JoystickButton(m_leftJoy, 4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_leftButton1.whenPressed(new InstantCommand(new SwerveModule(Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN)::turnRight)).whenReleased(new InstantCommand(new SwerveModule(Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN)::stopRotation));
    m_leftButton1.whenPressed(new InstantCommand(new SwerveModule(Constants.LEFT_BACK_DRIVE_CAN, Constants.LEFT_BACK_ANGLE_CAN)::turnRight)).whenReleased(new InstantCommand(new SwerveModule(Constants.LEFT_BACK_DRIVE_CAN, Constants.LEFT_BACK_ANGLE_CAN)::stopRotation));

    m_leftButton2.whenPressed(new InstantCommand(new SwerveModule(Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN)::turnLeft)).whenReleased(new InstantCommand(new SwerveModule(Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN)::stopRotation));
    m_leftButton2.whenPressed(new InstantCommand(new SwerveModule(Constants.LEFT_BACK_DRIVE_CAN, Constants.LEFT_BACK_ANGLE_CAN)::turnLeft)).whenReleased(new InstantCommand(new SwerveModule(Constants.LEFT_BACK_DRIVE_CAN, Constants.LEFT_BACK_ANGLE_CAN)::stopRotation));

    m_leftButton3.whenPressed(new InstantCommand(m_swerveDrivebase.mRightFrontModule::setZeroHeading));
    m_leftButton3.whenPressed(new InstantCommand(m_swerveDrivebase.mLeftBackModule::setZeroHeading));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public void teleopInit(){
    // final Command swerveDriveCommand = new RunCommand(() -> m_swerveDrivebase.holonomicDrive(m_leftJoy.getY(), m_leftJoy.getX(), m_rightJoy.getX()));
    // m_swerveDrivebase.setDefaultCommand(swerveDriveCommand);
}

  
}
