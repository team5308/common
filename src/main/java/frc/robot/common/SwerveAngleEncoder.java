package frc.robot.common;

public interface SwerveAngleEncoder {
    /**
     * Set offset
     * @param s_offset raw offset value
     */
    public abstract void setOffset(double s_offset);

    /**
     * set current position as zero
     */
    public abstract void setZero();


    /**
     * 
     * @return position in 0 deg ~ 359.9 deg
     */
    public abstract double getPosition();

    /**
     * 
     * @return raw sensor output
     */
    public abstract double getRawOutput();
}
