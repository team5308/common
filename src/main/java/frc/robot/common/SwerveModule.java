// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private Translation2d moduleLoc;
    
    public SwerveModule(int driveMotorID, int angleMotorID, Translation2d sModuleLoc) {

        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);
        this.moduleLoc  = sModuleLoc;

        angleMotor.setSelectedSensorPosition(0);

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        angleMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.config_kP(0, Constants.kAngleP);
        angleMotor.config_kI(0, Constants.kAngleI);
        angleMotor.config_kD(0, Constants.kAngleD);

        driveMotor.config_kP(0, Constants.kDriveP);
        driveMotor.config_kI(0, Constants.kDriveI);
        driveMotor.config_kD(0, Constants.kDriveD);
        driveMotor.config_kF(0, Constants.kDriveF);
    }

    public Translation2d getModulePosition() {
        return this.moduleLoc;
    }

    private double convertDeltaUnitToAngle(double deltaPosition) {
        return (deltaPosition * 360.0) / (Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio);
    }

    private double convertDeltaAngleToUnit(double deltaAngle) {
        return (deltaAngle * (Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio)) / 360.0;
    }

    public double getHeading() {
        double deltaPosition = angleMotor.getSelectedSensorPosition();
        double deltaAngle = convertDeltaUnitToAngle(deltaPosition);
        return keepWithin360deg(deltaAngle);
    }

    public static double keepWithin360deg(double angle) {
        while (angle >= 360.0) { angle -= 360.0;}
        while (angle  <   0.0) { angle += 360.0;}
        return angle;
    }
    
    public void set(double heading, double drive) {
        this.set(heading, drive, true);
    }

    public void set(double heading, double drive, boolean EnableDriveBackwards) {

    }

    public void setDrivePercent(double percentOutput) {
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    private void setModuleHeading(Rotation2d rotate) {
        return ;
    }
}
