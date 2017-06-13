package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.PidSource;
import edu.flash3388.flashlib.robot.PidType;
import edu.flash3388.flashlib.robot.actions.PidRotationActionPart;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;

public class TargetRotationActionPart extends PidRotationActionPart{

	private DoubleDataSource kp, ki, kd, margin;
	
	public TargetRotationActionPart(DoubleDataSource kp, DoubleDataSource ki, DoubleDataSource kd, DoubleDataSource margin) {
		super(new PidSource(){
			@Override
			public double pidGet() {
				return Robot.getInstance().getAngleToTarget();
			}
			@Override
			public PidType getType() {
				return PidType.Displacement;
			}
		}, kp.get(), ki.get(), kd.get(), ()->0.0, margin.get());
		
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.margin = margin;
	}

	@Override
	protected void initialize() {
		getPidController().setPID(kp.get(), ki.get(), kd.get());
		setRotationMargin(margin.get());
		
		super.initialize();
	}
}
