package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.sbc.IterativeSbc;

public abstract class RoboTestIterativeBot extends IterativeSbc{

	public static final byte STATE_OPERATION = 0x3;
	
	@Override
	protected void stateInit(byte state) {
		switch (state) {
			case STATE_AUTONOMOUS:
				mappingInit();
				break;
			case STATE_OPERATION:
				operationInit();
				break;
			case STATE_TELEOP:
				teleopInit();
				break;
		}
	}
	@Override
	protected void statePeriodic(byte state) {
		switch (state) {
			case STATE_AUTONOMOUS:
				mappingPeriodic();
				break;
			case STATE_OPERATION:
				operationPeriodic();
				break;
			case STATE_TELEOP:
				teleopPeriodic();
				break;
		}
	}

	protected abstract void teleopInit();
	protected abstract void mappingInit();
	protected abstract void operationInit();
	
	protected abstract void teleopPeriodic();
	protected abstract void mappingPeriodic();
	protected abstract void operationPeriodic();
}
