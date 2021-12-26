// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class MagEncoder {
    private CANSparkMax mSensor;
    private CANEncoder mCanEncoder;

    public MagEncoder(int CANSparkMaxPort) {
        mSensor = new CANSparkMax(CANSparkMaxPort, MotorType.kBrushed);
        mSensor.getEncoder()
    }

    
}
