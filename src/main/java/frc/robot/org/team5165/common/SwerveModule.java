package frc.robot.org.team5165.common;

import javax.swing.plaf.synth.SynthStyle;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.org.team5165.common.SwerveEncoders.SAECANCoder;
import frc.robot.org.team5165.common.SwerveEncoders.SAEMagEncoderTSRX;
import frc.robot.org.team5165.common.SwerveEncoders.SwerveAngleEncoder;

/** Add your docs here. */
public class SwerveModule {

    public TalonFX driveMotor;
    public TalonFX angleMotor;
    public SwerveAngleEncoder angleEncoder;
    public Translation2d moduleLoc;
    public String name;
    public double angleOffset;

    public double start;

    public SwerveModule(Translation2d sModuleLoc, int driveMotorID, int angleMotorID, int encoderID, double s_offset)
    {
        this(sModuleLoc, driveMotorID, angleMotorID, encoderID, false, s_offset);
    }

    public SwerveModule(Translation2d sModuleLoc, int driveMotorID, int angleMotorID, int encoderID, boolean isCANCoder, double s_offset) {

        moduleLoc  = sModuleLoc;

        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        if(isCANCoder) {
            angleEncoder = new SAECANCoder(encoderID);
        } else {
            angleEncoder = new SAEMagEncoderTSRX(encoderID);
        }

        angleEncoder.setOffset(s_offset);        

        angleMotor.setInverted(TalonFXInvertType.Clockwise);
        driveMotor.setInverted(TalonFXInvertType.Clockwise);

        
        TalonSRX mTalonSRX = new TalonSRX(encoderID);
        mTalonSRX.configFactoryDefault();
        // mTalonSRX.ConfigRe(SensorTerm.Sum0, FeedbackDevice.CTRE_MagEncoder_Absolute);
        mTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        mTalonSRX.configFeedbackNotContinuous(false, 1000);
        angleMotor.configRemoteFeedbackFilter(mTalonSRX.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        
        angleOffset = s_offset;
        
        angleMotor.configSelectedFeedbackCoefficient(1);

        // angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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

        start = angleMotor.getSelectedSensorPosition();
    }

    public void setName(String name) {
        this.name = name;
    }

    public Translation2d getModulePosition() {
        return this.moduleLoc;
    }

    public double getModuleHeading() {
        return angleMotor.getSelectedSensorPosition();
    }

    public void setState(SwerveModuleState targetState)
    {
        this.setState(targetState, true);
    }

    public void setState(SwerveModuleState targetState, boolean optimized) {
        if (optimized) {
            System.out.println(name + " opt from: " + targetState.toString());
            targetState = SwerveModuleState.optimize(targetState,
                Rotation2d.fromDegrees(getHeading()));
                System.out.println("to: " + targetState.toString());
        }
        setSpeed(targetState.speedMetersPerSecond);
        setAngle(targetState.angle);
        // System.out.println(name + " Speed: " + targetState.speedMetersPerSecond + " Angle: " + targetState.angle + " current heading: " + getModuleHeading());
    }

    public void setSpeed(double speedMetersPerSec) {
        driveMotor.set(ControlMode.PercentOutput, speedMetersPerSec);
    }

    /**
     *  to-do: Need to improve angle control algorithm
     * @param angle
     */
    public void setAngle(Rotation2d angle) {
        double deltaDegree = angle.getDegrees() - getHeading();
        // double deltaDegree = angle.getDegrees();
        if(Math.abs(deltaDegree) >= 180) {
            if(deltaDegree < 0) {
                deltaDegree += 360;
            } else {
                deltaDegree -= 360;
            }
        }
        System.out.println(name + " " +deltaDegree+ " " + getHeading() + " " + angleMotor.getSelectedSensorPosition());
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition() + deltaDegree* (4096.0 / 360.0));
    }

    public void setAngle() {
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition());
    }

    public double getHeading() {
        return normalizeDegAngle((angleMotor.getSelectedSensorPosition() - angleOffset) / 4096.0 * 360.0);
    }

    //For built-in encoder
    public double convertDeltaUnitToAngle(double deltaPosition){
        return (deltaPosition * 360.0)/(Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio);
    }

    //For built-in encoder
    public double convertDeltaAngleToUnit(double deltaAngle){
        return (deltaAngle * (Constants.kAngleEncoderTicksPerRotation * Constants.kEncoderGearRatio)) / 360.0;
    }

    public static double normalizeDegAngle(double angle) {
        double r = angle - 360.0 * ((int) (angle / 360) + (angle < 0 ? -1 : 0));
        return (r - 360.0) >= 0 ? r - 360.0 : r;
    }
}
