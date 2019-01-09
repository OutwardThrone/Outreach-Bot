/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2976.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import util.PIDSource;
import util.SimplePID;

import java.io.FileNotFoundException;
import java.io.IOException;

import org.usfirst.frc.team2976.robot.Robot;

import PIDStats.GatherStatistics;

/**
 * An example command.  You can replace me with your own command.
 */
public class TurnDegrees extends Command {
	private int clockCounter=0;
	private double angle;
	private final double CLOCK_MAX = 5;
	private boolean isFinished = false;
	private double error;
	private GatherStatistics stats;
	private Timer timer;
	private SimplePID turnPID;
	private PIDSource navxSource;
	
	public int tolerance = 5;
	
	private double kP = 0.01;
	private double kI = 0;
	private double kD = 0;
	
	double output;
	
	
	public TurnDegrees(double angle) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.m_drive);
		timer = new Timer();
		this.angle = angle + Robot.navx.getAngle();
		try {
			stats = new GatherStatistics();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("\ncouldn't make stats\n");
		}
		
		
		navxSource = new PIDSource() {
			public double getInput() {
				return Robot.navx.getAngle();
			}
		};
		
		turnPID = new SimplePID(navxSource, this.angle, kP, kI, kD);
		turnPID.setOutputLimits(-1, 1);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		timer.reset();
		timer.start();
		turnPID.resetPID();
		output = 0;
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		output = turnPID.compute();
		if (Math.abs(turnPID.getError()) < tolerance) {
			clockCounter++;
			if (clockCounter >= CLOCK_MAX) {
				isFinished = true;
			}
			
		} else {
			clockCounter = 0;
		}
		Robot.m_drive.tankDrive(-output, output);
		
		//Robot.smartDashboardAddData(timer.get(), navx.getAngle(), Robot.m_drive.ouput);
		stats.writeNewData(timer.get(), Robot.navx.getAngle(), output, turnPID.getError());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		//Robot.m_drive.turnController.disable();
		Robot.m_drive.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
