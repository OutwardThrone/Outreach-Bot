package util;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import org.usfirst.frc.team2976.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import java.util.Timer;
import java.util.TimerTask;

/**
 * The RPS (Robot Positioning system) attempts to give the most accurate
 * information about the current heading of the robot
 * 
 * This class will run three threads that will simultaneously calculate,
 * integrate and filter data from the following sensors:
 * 
 * Sensors: Wheel Encoders on Mecanum drive (this will be more complicated than
 * normal) ADNS Laser Tracker, a mouse, NAVX: experimental data double
 * integration of acceleration
 * 
 * @author NeilHazra
 */

public class RPS {
	public AHRS ahrs;

	double startingX;
	double startingY;
	double targetX;
	double targetY;

	public RPS(double targetX, double targetY) {
		this(targetX, targetY, 20);
	}
	
	public RPS() {
		ahrs = new AHRS(SPI.Port.kMXP);
	}

	public RPS(double targetX, double targetY, int ultrasonicMeasurementFrequencyInMillis) {
		ahrs = new AHRS(SPI.Port.kMXP);

		this.targetX = targetX;
		this.targetY = targetY;
	}

	public void reset() {
	 ahrs.reset();
	}

	// Will return degrees
	public double getAngle() {
		return ahrs.getAngle();
	}

}