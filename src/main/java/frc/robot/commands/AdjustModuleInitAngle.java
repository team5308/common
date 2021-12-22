// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivebase;

public class AdjustModuleInitAngle extends CommandBase {
  /** Creates a new AdjustModuleInitAngle. */
  SwerveDrivebase m_swerveDrivebase;

  XboxController m_xboxController;
  public static int num_of_mod = 0;
  public static XboxController m_XboxController = new XboxController(0);
  public AdjustModuleInitAngle(SwerveDrivebase m_swerveDrivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrivebase);

    this.m_swerveDrivebase = m_swerveDrivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Angle Adjust Module #", num_of_mod)
    if(m_XboxController.getBButtonReleased()) {
      num_of_mod++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrivebase.setDriveCommand();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_XboxController.getYButtonPressed();
  }
}
