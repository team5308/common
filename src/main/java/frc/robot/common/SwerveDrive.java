// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SwerveDrive {
    public final SwerveModule[] mSwerveModules;
    public final SwerveDriveKinematics mKinematics;
    public final Translation2d mCenterOfRotation;

    public SwerveDrive(SwerveModule... swerveModules) {
        this(new Translation2d(0, 0), swerveModules);
    }

    public SwerveDrive(Translation2d centerOfRotation, SwerveModule... swerveModules) {
        if (swerveModules.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        
        mSwerveModules = swerveModules;
        mKinematics = new SwerveDriveKinematics(
            (Translation2d[]) Arrays.asList(mSwerveModules)
                .stream()
                .map((module) -> module.getModulePosition())
                .toArray()
            );
            mCenterOfRotation = centerOfRotation;
    }

    public void setMotion(ChassisSpeeds desiredSpeed) {
        setMotion(desiredSpeed, mCenterOfRotation);
    }

    public void setMotion(ChassisSpeeds desireSpeeds, Translation2d centerOfRotationMeters) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(desireSpeeds, centerOfRotationMeters);
        
        for(int i = 0; i < moduleStates.length; i++) {
            mSwerveModules[i].setState(moduleStates[i]);
        }
    }
}
