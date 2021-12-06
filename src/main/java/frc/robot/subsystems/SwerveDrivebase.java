// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class SwerveDrivebase extends SubsystemBase {
  SwerveModule[] mSwerveModules;
  //TODO: change public to private
  private SwerveModule mRightFrontModule;
  private SwerveModule mLeftFrontModule;
  private SwerveModule mLeftBackModule;
  private SwerveModule mRightBackModule;

  private AHRS m_navX;


  /** Creates a new SwerveDrivebase. */
  public SwerveDrivebase() {
    this.mRightFrontModule = new SwerveModule(Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN);
    this.mLeftFrontModule = new SwerveModule(Constants.LEFT_FRONT_DRIVE_CAN, Constants.LEFT_FRONT_ANGLE_CAN);
    this.mLeftBackModule = new SwerveModule(Constants.LEFT_BACK_DRIVE_CAN, Constants.LEFT_BACK_ANGLE_CAN);
    this.mRightBackModule = new SwerveModule(Constants.RIGHT_BACK_DRIVE_CAN, Constants.RIGHT_BACK_ANGLE_CAN);

    this.m_navX = new AHRS(SPI.Port.kMXP);

    //navX calibration
    m_navX.calibrate();
    m_navX.reset();

    SwerveModule[] mSwerveModules = new SwerveModule[]{
      mRightFrontModule,
      mLeftFrontModule,
      mLeftBackModule,
      mRightBackModule
    };
    this.mSwerveModules = mSwerveModules;
  }

  
  public void holonomicDrive(double forward, double strafe, double rotation){

    //Field Oriented Driving
    double temp = forward * Math.cos(getDrivetrainHeading()) + strafe * Math.sin(getDrivetrainHeading());
    strafe = -forward * Math.sin(getDrivetrainHeading()) + strafe * Math.cos(getDrivetrainHeading());
    forward = temp;
    rotation *= Math.abs(rotation);

    //Traditional Driving
    // forward *= Math.abs(forward);
    // strafe *= Math.abs(strafe);
    // rotation *= Math.abs(rotation);

    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("rotation", rotation);

    double L = Constants.WHEELBASE;
    double W = Constants.TRACKWIDTH;
    double R = Math.sqrt(Math.pow(L,2) + Math.pow(W,2));

    //a front right
    //b front left
    //c back left
    //d back right
    double a = strafe - rotation * (L/R);
    double b = strafe + rotation * (L/R);
    double c = forward - rotation * (W/R);
    double d = forward + rotation * (W/R);

    double[] angles = new double[]{
      Math.atan2(b,c)*180 / Math.PI,
      Math.atan2(b,d)*180 / Math.PI,
      Math.atan2(a,d)*180 / Math.PI,
      Math.atan2(a,c)*180 / Math.PI,
    };

    double[] speeds = new double[]{
      Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2)),
      Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2)),
      Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2)),
      Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2))
    };

    double max = speeds[0];

    for(double speed: speeds){
      if (speed > max){
        max = speed;
      }

    if(max>1){
      for(int i=0;i<4;i++){
        speeds[i] = speeds[i]/max;
      }
    }

    for(int i=0;i<4;i++){
      if (Math.abs(forward) > 0.01|| Math.abs(strafe) > 0.01 || Math.abs(rotation) > 0.01){
        mSwerveModules[i].set(angles[i],speeds[i]);
      }
      else{
        mSwerveModules[i].set(mSwerveModules[i].getHeading(), speeds[i]);
      }
    }

    
    }

  }
  

  public double getDrivetrainHeading(){
    SmartDashboard.putNumber("Drivetrain Heading", SwerveModule.keepWithin360deg(m_navX.getAngle()));
    return SwerveModule.keepWithin360deg(m_navX.getAngle());
  }

  public double getVelocityX(){
    SmartDashboard.putNumber("X Dist", m_navX.getVelocityX());
    return m_navX.getVelocityX();
  }

  public double getVelocityY(){
    SmartDashboard.putNumber("Y Dist", m_navX.getVelocityY());
    return m_navX.getVelocityY();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDrivetrainHeading();
    Timer.delay(2);
  }
}
