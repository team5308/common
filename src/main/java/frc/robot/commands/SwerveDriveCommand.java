// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID;

import frc.robot.subsystems.SwerveDrivebase;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  SwerveDrivebase m_swerveDrivebase;
  ForwardStrafeRotationSupplier m_supplier;

  XboxController m_xboxController;

  public SwerveDriveCommand(SwerveDrivebase m_swerveDrivebase, XboxController m_xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrivebase);

    this.m_swerveDrivebase = m_swerveDrivebase;
    // this.m_supplier = m_supplier;
    this.m_xboxController = m_xboxController;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_swerveDrivebase.holonomicDrive(-m_xboxController.getY(GenericHID.Hand.kLeft), m_xboxController.getX(GenericHID.Hand.kLeft), m_xboxController.getX(GenericHID.Hand.kRight));
  }

  @Override
  public boolean isFinished() {
     return false;
  }

}
