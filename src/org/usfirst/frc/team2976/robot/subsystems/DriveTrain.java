package org.usfirst.frc.team2976.robot.subsystems;

import org.usfirst.frc.team2976.robot.Robot;
import org.usfirst.frc.team2976.robot.RobotMap;
import org.usfirst.frc.team2976.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import util.SimplePID;

public class DriveTrain extends Subsystem{
	private WPI_TalonSRX leftFront, leftBack, rightFront, rightBack;
	private Encoder encoder;
	
	public DriveTrain() {
		
		leftFront = new WPI_TalonSRX(RobotMap.leftFrontMotorPort);
		leftBack = new WPI_TalonSRX(RobotMap.leftBackMotorMotorPort);
		rightFront = new WPI_TalonSRX(RobotMap.rightFrontMotorPort);
		rightBack = new WPI_TalonSRX(RobotMap.rightBackMotorPort);
		
		rightFront.setInverted(true);
		rightBack.setInverted(true);
		
		encoder = new Encoder(1, 0);
		
		//encoder.getRaw(); // <- use this
	}
	
	public double encoderOutput() {
		return encoder.getRaw();
	}
	
	public void rotate(double angle) {
		//this.basePower = basePower;
		/*turnController.setPID(kP, kI, kD);
		turnController.reset();
		turnController.setSetpoint(angle);
		turnController.enable();*/
		//turnControler = new SimplePID(navx, angle, kP, kI, kD);
		
	}
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		setDefaultCommand(new TankDrive());
	}
	
	public void setPower(ControlMode mode, double leftPower, double rightPower) {
		
		leftFront.set(mode, leftPower);
		leftBack.set(mode, leftPower);
		rightFront.set(mode, rightPower);
		rightBack.set(mode, rightPower);
		
		
		
	}
	
	public void arcadeDrive(double turn, double speed) {
		//drive.arcadeDrive(speed, turn);
	}
	
	public void tankDrive(double leftPower, double rightPower) {
		//drive.tankDrive(leftPower, rightPower);
		leftFront.set(leftPower);
		leftBack.set(leftPower);
		rightFront.set(rightPower);
		rightBack.set(rightPower);
		
	}
	
}
