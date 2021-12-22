// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AdjustModuleInitAngle;
import frc.robot.commands.AutonomousForward;
import frc.robot.commands.ForwardStrafeRotationSupplier;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwitchModuleDefaultCmd;
import frc.robot.subsystems.SwerveDrivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivebase m_swerveDrivebase = new SwerveDrivebase();

  private final AutonomousForward m_autoForward = new AutonomousForward();

  private final XboxController m_xboxController = new XboxController(0);


  // private final ForwardStrafeRotationSupplier m_supplier = new ForwardStrafeRotationSupplier(m_leftJoy, m_rightJoy);
  // private final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_swerveDrivebase, m_xboxController);



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
    // port 3 : Xbox 'X'
    (new JoystickButton(m_xboxController, 3)).whenPressed(new SwitchModuleDefaultCmd(m_swerveDrivebase));
    // port 1 : Xbox 'A'
    (new JoystickButton(m_xboxController, 1)).whenReleased(new InstantCommand(){
        public void initialize()
        {
          AdjustModuleInitAngle.num_of_mod++;
          AdjustModuleInitAngle.num_of_mod %= 4;
        }
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoForward;
  }

  public void autonomousInit(){

  }

  public void teleopInit(){
    // m_supplier.schedule();
    // m_swerveDrivebase.setDefaultCommand(m_swerveDriveCommand);
  }

  public void disabledInit() {}
  
}
