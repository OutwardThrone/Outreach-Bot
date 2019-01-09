package org.usfirst.frc.team2976.robot.commands;

import org.usfirst.frc.team2976.robot.Robot;
import org.usfirst.frc.team2976.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class TankDrive extends Command {
	private double speed = 0;
	private double turn = 0;
	
	public TankDrive() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.m_drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.m_drive.tankDrive(0, 0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		/*speed = Robot.m_oi.stick.getRawAxis(RobotMap.leftYAxis);
		turn = Robot.m_oi.stick.getRawAxis(RobotMap.rightXAxis);
				
		Robot.m_drive.arcadeDrive(turn, speed);*/
		
		System.out.println(Robot.m_drive.encoderOutput());
		
		Robot.m_drive.tankDrive(-Robot.m_oi.stick.getRawAxis(RobotMap.leftYAxis), -Robot.m_oi.stick.getRawAxis(RobotMap.rightYAxis));
		
		//Robot.m_drive.tankDrive(0.4, 0.4);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.m_drive.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
	
}
