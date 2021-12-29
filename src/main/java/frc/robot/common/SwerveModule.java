package frc.robot.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.common.SwerveEncoders.SAECANCoder;
import frc.robot.common.SwerveEncoders.SAEMagEncoderTSRX;
import frc.robot.common.SwerveEncoders.SwerveAngleEncoder;

/** Add your docs here. */
public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private SwerveAngleEncoder angleEncoder;
    private Translation2d moduleLoc;

    public SwerveModule(Translation2d sModuleLoc, int driveMotorID, int angleMotorID, int encoderID)
    {
        this(sModuleLoc, driveMotorID, angleMotorID, encoderID, false);
    }

    public SwerveModule(Translation2d sModuleLoc, int driveMotorID, int angleMotorID, int encoderID, boolean isCANCoder) {

        moduleLoc  = sModuleLoc;

        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        if(isCANCoder) {
            angleEncoder = new SAECANCoder(encoderID);
        } else {
            angleEncoder = new SAEMagEncoderTSRX(encoderID);
        }
        

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

    public double getModuleHeading() {
        return angleEncoder.getPosition();
    }

    public void setState(SwerveModuleState targetState)
    {
        this.setState(targetState, true);
    }

    public void setState(SwerveModuleState targetState, boolean optimized) {
        if (optimized) {
            targetState = SwerveModuleState.optimize(targetState,
                Rotation2d.fromDegrees(getModuleHeading()));
        }
        setSpeed(targetState.speedMetersPerSecond);
        setAngle(targetState.angle);
    }

    public void setSpeed(double speedMetersPerSec) {
        driveMotor.set(ControlMode.Velocity, speedMetersPerSec);
    }

    /**
     *  to-do: Need to improve angle control algorithm
     * @param angle
     */
    public void setAngle(Rotation2d angle) {
        double deltaDegree = angle.getDegrees() - getModuleHeading();
        angleMotor.set(ControlMode.Position, convertDeltaAngleToUnit(deltaDegree) + angleMotor.getSelectedSensorPosition());
    }

    //For built-in encoder
  private double convertDeltaUnitToAngle(double deltaPosition){
    return (deltaPosition * 360.0)/(Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio);
  }

  //For built-in encoder
  private double convertDeltaAngleToUnit(double deltaAngle){
    return (deltaAngle * (Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio)) / 360.0;
  }
}
