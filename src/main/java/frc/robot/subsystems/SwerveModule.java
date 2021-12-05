// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;


import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  public TalonFX driveMotor;
  public TalonFX angleMotor;
  private double moduleInitialPosition;
  //Heading is in degree, read from absolute position of external CANCoder
  private double moduleInitialHeading = 0;
  private double calibratedInitialHeading = 0;
  //offset is in degree, negative or positive, absolute value from 0 to 360.
  private double offset = 0;

  //construct an CANCoder
  private CANCoder canCoder;

  //Construct a module without a CANCoder, encoder from angle motor is used directly, not encouraged. Before each match, modules should be aligned manually.
  //TODO: Will be removed once CANCoder is installed onto the module.
  public SwerveModule(int driveMotorID, int angleMotorID) {
    this.driveMotor = new TalonFX(driveMotorID);
    this.angleMotor = new TalonFX(angleMotorID);
    moduleInitialPosition = angleMotor.getSelectedSensorPosition();


    angleMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Brake);
  
    // TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

    // // angleTalonFXConfiguration.slot0.kP = Constants.kAngleP;
    // // angleTalonFXConfiguration.slot0.kI = Constants.kAngleI;
    // // angleTalonFXConfiguration.slot0.kD = Constants.kAngleD;


    // TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

    // driveTalonFXConfiguration.slot0.kP = Constants.kDriveP;
    // driveTalonFXConfiguration.slot0.kI = Constants.kDriveI;
    // driveTalonFXConfiguration.slot0.kD = Constants.kDriveD;
    // driveTalonFXConfiguration.slot0.kF = Constants.kDriveF;

  }

    //Construct a module with a CANCoder. Offset is in degree. Offset is the value of the encoder when the wheel points to a desired direction.
    public SwerveModule(int driveMotorID, int angleMotorID, int encoderID, double offset) {
      this.driveMotor = new TalonFX(driveMotorID);
      this.angleMotor = new TalonFX(angleMotorID);
      this.moduleInitialPosition = canCoder.getPosition();

      //declare an CANCoder
      this.canCoder = new CANCoder(encoderID);

      //Default value, to be changed with absolute readings from CANCoder.
      canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.valueOf(0)); //CANCoder reading is from 0 to 360.
      this.moduleInitialHeading = canCoder.getAbsolutePosition();

      //Read from Constants.
      this.offset = offset;
      calibratedInitialHeading = this.moduleInitialHeading - this.offset;
  
  
      angleMotor.setNeutralMode(NeutralMode.Brake);
      driveMotor.setNeutralMode(NeutralMode.Brake);
    
      TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();
  
      angleTalonFXConfiguration.slot0.kP = Constants.kAngleP;
      angleTalonFXConfiguration.slot0.kI = Constants.kAngleI;
      angleTalonFXConfiguration.slot0.kD = Constants.kAngleD;

      // Use the CANCoder as the remote sensor for the primary TalonFX PID
      angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
      angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
      angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
      angleMotor.configAllSettings(angleTalonFXConfiguration);
  
  
      TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
  
      driveTalonFXConfiguration.slot0.kP = Constants.kDriveP;
      driveTalonFXConfiguration.slot0.kI = Constants.kDriveI;
      driveTalonFXConfiguration.slot0.kD = Constants.kDriveD;
      driveTalonFXConfiguration.slot0.kF = Constants.kDriveF;
  
    }

    //For built-in encoder
  public double convertDeltaUnitToAngle(double deltaPosition){
    return (deltaPosition * 360.0)/(Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio);
  }


  public double convertDeltaAngleToUnit(double deltaAngle){
    return (deltaAngle * (Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio)) / 360.0;
  }


  public double getHeading() {
    double deltaPosition = angleMotor.getSelectedSensorPosition() - moduleInitialPosition;
    double deltaAngle = convertDeltaUnitToAngle(deltaPosition) + calibratedInitialHeading;
    return keepWithin360deg(deltaAngle);
  }

  //For CANCoder
  // public double convertDeltaUnitToAngle(double deltaPosition){
  //   return (deltaPosition * 360.0)/(Constants.kACANCoderTicksPerRotation * Constants.kCANCoderGearRatio);
  // }
  

  // public double convertDeltaAngleToUnit(double deltaAngle){
  //   return (deltaAngle * (Constants.kCANCoderTicksPerRotation * Constants.kCANCoderGearRatio)) / 360.0;
  // }
  

  // public double getHeading() {
  //   double deltaPosition = canCoder.getPosition() - moduleInitialPosition;
  //   double deltaAngle = convertDeltaUnitToAngle(deltaPosition) + calibratedInitialHeading;
  //   return keepWithin360deg(deltaAngle);
  // }


  //For built-in encoder
  //rotate the module to an angle (0 to 360)
  public void setModuleAngle(double desiredPosition){
    double currentPosition = getHeading();
    double deltaAngle = desiredPosition - currentPosition;

    double deltaUnit = convertDeltaAngleToUnit(deltaAngle);

    angleMotor.setSelectedSensorPosition((angleMotor.getSelectedSensorPosition() + deltaUnit)); //TODO: Change relationship of feedback&feed device

  }

  //For CANCoder
  // public void setModuleAngle(double desiredPosition){
  //   double currentPosition = getHeading();
  //   double deltaAngle = desiredPosition - currentPosition;

  //   double deltaUnit = convertDeltaAngleToUnit(deltaAngle);

  //   angleMotor.setSelectedSensorPosition((canCoder.getPosition() + deltaUnit));

  // }


  public void set(double heading, double drive){
    if(shouldDriveBackwards(heading, getHeading())){
      setHeadingTarget(heading+180);
      setDrivePercent(-drive);
    }

    else{
      setHeadingTarget(heading);
      setDrivePercent(drive);
    }
  }


  public static boolean shouldDriveBackwards(double goalAngle, double currentAngle){
    goalAngle = keepWithin360deg(goalAngle);
    currentAngle = keepWithin360deg(currentAngle);
    double reversedAngle = keepWithin360deg(goalAngle);
    double angleDifference = Math.abs(goalAngle - currentAngle);
    double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);

    if (angleDifference>180){
      angleDifference = 360-angleDifference;
    }
    else{}

    return reversedAngleDifference < angleDifference;
  }


  public static double keepWithin360deg(double angle){
    while (angle >= 360.0){angle -= 360.0;}
    while (angle < 0.0){angle += 360.0;}
    return angle;
  }


  public void setDrivePercent(double percentOutput){
    driveMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  

  public void setHeadingTarget(double degrees){
    double target = degrees;
    double position = getHeading();

    while(position-target>180){
      target+=360;
    }
    while(target-position>180){
      target-=360;
    }

    setModuleAngle(target);
  }

}
