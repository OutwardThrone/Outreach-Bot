package org.usfirst.frc.team2976.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoExample extends CommandGroup{
	
	public AutoExample() {
		addSequential(new TurnDegrees(90));
	}

}
