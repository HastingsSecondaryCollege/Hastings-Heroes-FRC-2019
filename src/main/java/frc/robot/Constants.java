package frc.robot;

public class Constants {

	/** which Talon on CANBus */
	//public static final int kTalonID = 3;  //rear left

	/**
	 * How many sensor units per rotation. Using CTRE Magnetic Encoder.
	 * 
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	/*
	 * Convert pulses per rotation to pulses per meter.  So that we can do way points in meters.
	 * 2(pi)R for 6" wheels at 1440 pulses per rotation.
	 * 2.08865 rotations per meter.
	 * 2.08865 x 1440 pulses/meter 
	 */
	public static final double kSensorUnitsPerRotation = 4096; //3007.656; // (2.08865 * 1440); //convert rotations to meters //2970; //2880; /*4096;*/

	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;
	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Base trajectory period to add to each individual trajectory point's
	 * unique duration. This can be set to any value within [0,255]ms.
	 */
	public static final int kBaseTrajPeriodMs = 0;

	/**
	 * Motor deadband, set to 1%.
	 */
	public static final double kNeutralDeadband = 0.01;
}
