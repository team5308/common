// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveDrivebase;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  SwerveDrivebase m_swerveDrivebase;
  ForwardStrafeRotationSupplier m_supplier;

  public SwerveDriveCommand(SwerveDrivebase m_swerveDrivebase, ForwardStrafeRotationSupplier m_supplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrivebase);

    this.m_swerveDrivebase = m_swerveDrivebase;
    this.m_supplier = m_supplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_swerveDrivebase.holonomicDrive(m_supplier.getForward(), m_supplier.getStrafe(), m_supplier.getRotation());
  }

  @Override
  public boolean isFinished() {
     return true;
  }

}
