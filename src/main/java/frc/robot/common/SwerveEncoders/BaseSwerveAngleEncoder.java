package frc.robot.common.SwerveEncoders;

import frc.robot.common.SwerveEncoders.SwerveAngleEncoder;

public abstract class BaseSwerveAngleEncoder implements SwerveAngleEncoder {
    public double mOffset = 0;
    public double mCof = 1;

    @Override
    public void setOffset(double s_offset) {
        this.mOffset = s_offset;
    }
    
    @Override
    public double getPosition() {
        return normalizeDegAngle((getRawOutput() - mOffset) * mCof);
    }

    public static double normalizeDegAngle(double angle) {
        return angle - 360.0 * ((int) (angle / 360) + (angle < 0 ? -1 : 0));
    }
}
