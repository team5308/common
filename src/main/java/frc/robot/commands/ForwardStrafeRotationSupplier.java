// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ForwardStrafeRotationSupplier extends CommandBase {
  /** Creates a new ForwardStrafeRotationSupplier. */
  private double forward;
  private double strafe;
  private double rotation;

  private boolean overrideJoystickForward = false;
  private boolean overrideJoystickStrafe = false;
  private boolean overrideJoystickRotation = false;


  private Joystick m_leftJoy;
  private Joystick m_rightJoy;

  public ForwardStrafeRotationSupplier(Joystick m_leftJoy, Joystick m_rightJoy) {
    this.m_leftJoy = m_leftJoy;
    this.m_rightJoy = m_rightJoy;
  }

  public void updateRotation(double updateRotation){
    rotation = updateRotation;
    overrideJoystickRotation = true;
  }

  public void updateForward(double updateForward){
    forward = -updateForward;
    overrideJoystickForward = true;
  }

  public void updateStrafe(double updateStrafe){
    strafe = updateStrafe;
    overrideJoystickStrafe = true;
  }

  public double getForward(){
    return forward;
  }

  public double getStrafe(){
    return strafe;
  }

  public double getRotation(){
    return rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(overrideJoystickForward){
      
    }
    else{
      forward = -m_leftJoy.getY();
    }

    if(overrideJoystickStrafe){

    }
    else{
      strafe = m_leftJoy.getX();
    }

    if(overrideJoystickRotation){
     
    }
    else{
      rotation = m_rightJoy.getX();
    }
    overrideJoystickForward = false;
    overrideJoystickStrafe = false;
    overrideJoystickRotation = false;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
