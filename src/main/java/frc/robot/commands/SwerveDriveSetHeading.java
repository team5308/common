// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivebase;

public class SwerveDriveSetHeading extends CommandBase {
  /** Creates a new SwerveDriveSetHeading. */
  SwerveDrivebase m_swerveDrivebase;
  double desiredHeading;
  double angularSpeed;
  ForwardStrafeRotationSupplier m_supplier;

  public SwerveDriveSetHeading(SwerveDrivebase m_swerveDrivebase, ForwardStrafeRotationSupplier m_supplier, double desiredHeading, double angularSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrivebase);
    this.m_swerveDrivebase = m_swerveDrivebase;
    this.desiredHeading = desiredHeading;
    this.m_supplier = m_supplier;
  }

  private void setDrivetrainHeadingTarget(double degrees, double angularSpeed){
    double target = degrees;
    double position = m_swerveDrivebase.getDrivetrainHeading();

    while(position - target > 180){
      target += 360;
    }
    while(target - position > 180){
      target -= 360;
    }

    setDrivetrainAngleChange(target, angularSpeed);
  }


  private void setDrivetrainAngleChange(double angleChange, double angularSpeed){
    if(angleChange > 0){
      m_supplier.updateRotation(Math.abs(angularSpeed));;
    }
    else{
      m_supplier.updateRotation(-Math.abs(angularSpeed));

    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setDrivetrainHeadingTarget(desiredHeading, angularSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double allowedError = 5;
    if(Math.abs(desiredHeading-m_swerveDrivebase.getDrivetrainHeading())<allowedError){
      return true;
    }
    return false;
  }
}
