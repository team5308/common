package frc.robot.common.SwerveEncoders;

import com.ctre.phoenix.sensors.CANCoder;

public class SAECANCoder extends BaseSwerveAngleEncoder {

    private CANCoder cancoder;

    public SAECANCoder(int canID) {
        cancoder = new CANCoder(canID);
         
        /**  CANCoder has a default period of 10ms.
        * See https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html */
        // cancoder.setStatusFramePeriod(statusFrame, periodMs); 
    }

    @Override
    public void setZero() {
        setOffset(getRawOutput());
    }

    @Override
    public double getRawOutput() {
        return cancoder.getAbsolutePosition();
    }
    
}
