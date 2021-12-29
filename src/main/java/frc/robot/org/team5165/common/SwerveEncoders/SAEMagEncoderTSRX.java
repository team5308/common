package frc.robot.common.SwerveEncoders;

import java.util.logging.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SAEMagEncoderTSRX extends BaseSwerveAngleEncoder {

    private SensorCollection mSensorCollection;

    public SAEMagEncoderTSRX(int talonSRXID) {
        TalonSRX talonsrx = new TalonSRX(talonSRXID);
        ErrorCode er = talonsrx.setStatusFramePeriod(8, 10, 1000);
        
        if(er != ErrorCode.OK) {
            Logger.getAnonymousLogger()
                .severe(
                    String.format("TalonSRX (%d) ErrorCode: %d when set Status Frame", talonSRXID, er)
                    );;
        }

        mSensorCollection = talonsrx.getSensorCollection();
    }

    @Override
    public void setZero() {
        setOffset(getRawOutput());
    }

    @Override
    public double getRawOutput() {
        return mSensorCollection.getPulseWidthPosition();
    }
    
}
