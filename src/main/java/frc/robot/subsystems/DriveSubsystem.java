// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.crypto.dsig.XMLObject;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.org.team5165.common.SwerveDrive;
import frc.robot.org.team5165.common.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  public SwerveModule mSMrightfront;
  public SwerveModule mSMleftback;
  public SwerveDrive mSwerveDrive;

  public TalonSRX m13 = new TalonSRX(13);
  public TalonSRX m33 = new TalonSRX(33);

  public double speedFactor = 0.5;
  
  public XboxController mXboxController = new XboxController(0);

  public DriveSubsystem() {
    mSMrightfront = new SwerveModule(new Translation2d(0.36, 0.36), Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN, 13, 841);
    mSMleftback   = new SwerveModule(new Translation2d(-0.36, -0.36), Constants.LEFT_BACK_DRIVE_CAN,   Constants.LEFT_BACK_ANGLE_CAN,   33, 960);

    mSMleftback.setName("Left Back");
    mSMrightfront.setName("Right Front");

    mSwerveDrive = new SwerveDrive(mSMrightfront, mSMleftback);
  }

  @Override
  public void periodic() {
    double leftX = db(-mXboxController.getX(Hand.kLeft));
    double leftY = db(mXboxController.getY(Hand.kLeft));
    double v_rad = db(mXboxController.getX(Hand.kRight));
    if( Math.abs(leftX) > 0.05 || Math.abs(leftY) > 0.05 || Math.abs(v_rad) > 0.05) {
      ChassisSpeeds cSpeeds = new ChassisSpeeds(leftY, leftX, -v_rad);
      mSwerveDrive.setMotion(cSpeeds);
    } else {
      mSwerveDrive.setZeroSpeed();
    }
    // mSMleftback.angleMotor.set(ControlMode.PercentOutput, leftX);
    System.out.println(leftX);
    // mSMrightfront.setAngle(Rotation2d.fromDegrees(leftX * 180));
    // mSMrightfront.angleMotor.set(ControlMode.Position, mSMrightfront.start + mSMrightfront.convertDeltaAngleToUnit(360.0));
      // mSMrightfront.driveMotor.set(ControlMode.PercentOutput, leftX);
      // mSMleftback.driveMotor.set(ControlMode.PercentOutput, leftX);
      // mSMleftback.setAngle(Rotation2d.fromDegrees(0));
      // mSMrightfront.setAngle(Rotation2d.fromDegrees(0));

      SmartDashboard.putNumber("13_number", m13.getSelectedSensorPosition());
      SmartDashboard.putNumber("33_number", m33.getSelectedSensorPosition());


  }

  public static double db(double x)
  {
    return Math.abs(x) < 0.2 ? 0 : x;
  }
}
