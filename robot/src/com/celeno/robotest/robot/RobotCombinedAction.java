package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.SourceAction;
import edu.flash3388.flashlib.robot.actions.TankCombinedAction;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.robot.systems.TankDriveSystem;

public class RobotCombinedAction extends TankCombinedAction{

	private DoubleDataSource minSpeed, maxSpeed;
	
	public RobotCombinedAction(TankDriveSystem driveTrain, SourceAction positioning, SourceAction rotation, 
			DoubleDataSource minSpeed, DoubleDataSource maxSpeed) {
		super(driveTrain, positioning, rotation);
		this.minSpeed = minSpeed;
		this.maxSpeed = maxSpeed;
	}

	@Override
	protected void initialize() {
		setMaxSpeed(maxSpeed.get());
		setMinSpeed(minSpeed.get());
		super.initialize();
	}
	@Override
	protected void end() {
		super.end();
	}
}
