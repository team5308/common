// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveDrivebase extends SubsystemBase {
  SwerveModule[] mSwerveModules;
  private SwerveModule mRightFrontModule = new SwerveModule(Constants.RIGHT_FRONT_DRIVE_CAN, Constants.RIGHT_FRONT_ANGLE_CAN);
  private SwerveModule mLeftFrontModule = new SwerveModule(Constants.LEFT_FRONT_DRIVE_CAN, Constants.LEFT_FRONT_ANGLE_CAN);
  private SwerveModule mLeftBackModule = new SwerveModule(Constants.LEFT_BACK_DRIVE_CAN, Constants.LEFT_BACK_ANGLE_CAN);
  private SwerveModule mRightBackModule = new SwerveModule(Constants.RIGHT_BACK_DRIVE_CAN, Constants.RIGHT_BACK_ANGLE_CAN);


  /** Creates a new SwerveDrivebase. */
  public SwerveDrivebase() {
    SwerveModule[] mSwerveModules = new SwerveModule[]{
      mRightFrontModule,
      mLeftFrontModule,
      mLeftBackModule,
      mRightBackModule
    };
    this.mSwerveModules = mSwerveModules;


  }

  public void holonomicDrive(double forward, double strafe, double rotation){


    forward *= Math.abs(forward);
    strafe *= Math.abs(strafe);
    rotation *= Math.abs(rotation);

    //a front right
    //b front left
    //c back left
    //d back right
    double a = strafe - rotation * (Constants.WHEELBASE/Constants.TRACKWIDTH);
    double b = strafe + rotation * (Constants.WHEELBASE/Constants.TRACKWIDTH);
    double c = forward - rotation * (Constants.WHEELBASE/Constants.TRACKWIDTH);
    double d = forward - rotation * (Constants.WHEELBASE/Constants.TRACKWIDTH);

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

    for(int i=0;i<4;i++){
      speeds[i] = speeds[i]/max;
    }

    for(int i=0;i<4;i++){
      if (Math.abs(forward)< 0.01|| Math.abs(strafe)<0.01|| Math.abs(rotation)<0.01){
        angles[i] = mSwerveModules[i].getHeading();
      }
      mSwerveModules[i].set(angles[i], speeds[i]);
    }


    
    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
