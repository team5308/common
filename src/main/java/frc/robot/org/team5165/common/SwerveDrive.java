// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.org.team5165.common;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        // mKinematics = new SwerveDriveKinematics(
        //     (Translation2d[]) Arrays.asList(mSwerveModules)
        //         .stream()
        //         .map((module) -> module.getModulePosition())
        //         .toArray()
        //     );

        ArrayList<Translation2d> modulePositions = new ArrayList<Translation2d>();

        for(SwerveModule t2d : swerveModules) {
            modulePositions.add(t2d.getModulePosition());
        }

        Object[] objects = modulePositions.toArray();

        mKinematics = new SwerveDriveKinematics(Arrays.copyOf(objects, objects.length, Translation2d[].class));


        mCenterOfRotation = centerOfRotation;
    }

    public void setMotion(ChassisSpeeds desiredSpeed) {
        setMotion(desiredSpeed, mCenterOfRotation);
    }

    public void setMotion(ChassisSpeeds desireSpeeds, Translation2d centerOfRotationMeters) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(desireSpeeds, centerOfRotationMeters);
        
        for(int i = 0; i < moduleStates.length; i++) {
            mSwerveModules[i].setState(moduleStates[i]);
            // SmartDashboard.putNumber("" + i, mSwerveModules[i].getModuleHeading()); 
        }
    }

    public void setZeroSpeed() {
        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setSpeed(0);
        }
    }

}
