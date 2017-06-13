package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.RobotState;
import edu.flash3388.flashlib.robot.Scheduler;
import edu.flash3388.flashlib.robot.sbc.MotorSafetyHelper;
import edu.flash3388.flashlib.robot.sbc.SbcBot;
import edu.flash3388.flashlib.util.FlashUtil;
import edu.flash3388.flashlib.util.Log;

public abstract class RoboTestIterativeBot extends SbcBot{

	public static final byte STATE_OPERATION = 0x3;
	
	private boolean stop = false;
	
	@Override
	protected void startRobot() {
		Log log = FlashUtil.getLog();
		int safetyIterations = 0;
		robotInit();
		
		byte lastState = -1;
		byte state = STATE_DISABLED;
		while (!stop) {
			if(RobotState.inEmergencyStop()){
				log.save();
				log.log("NEW STATE - EMERGENCY STOP");
				disabledInit();
				
				while (!stop && RobotState.inEmergencyStop()) {
					disabledPeriodic();
					FlashUtil.delay(5);
				}
				continue;
			}
			
			state = getCurrentState();
			if(state != lastState)
				log.logTime("STATE DONE");
				
			if(state == STATE_DISABLED){
				if(state != lastState){
					lastState = state;
					
					log.save();
					log.logTime("NEW STATE - DISABLED");
					
					disabledInit();
					Scheduler.disableScheduler(true);
				}
				disabledPeriodic();
			}
			else if(state == STATE_TELEOP){
				if(state != lastState){
					lastState = state;
					
					log.save();
					log.logTime("NEW STATE - TELEOP");
					
					teleopInit();
					Scheduler.disableScheduler(false);
				}
				teleopPeriodic();
			}
			else if(state == STATE_OPERATION){
				if(state != lastState){
					lastState = state;
					
					log.save();
					log.logTime("NEW STATE - OPERATION");
					
					operationInit();
					Scheduler.disableScheduler(false);
				}
				operationPeriodic();
			}
			else if(state == STATE_AUTONOMOUS){
				if(state != lastState){
					lastState = state;
					
					log.save();
					log.logTime("NEW STATE - MAPPING");
					
					mappingInit();
					Scheduler.disableScheduler(false);
				}
				mappingPeriodic();
			}
			
			if(++safetyIterations > 4){
				MotorSafetyHelper.checkAll();
				safetyIterations = 0;
			}
			
			if(!getControlStation().isCSAttached()){
				log.reportWarning("CS CONNECTION LOST");
				while(!getControlStation().isCSAttached()) 
					FlashUtil.delay(10);
				log.logTime("CS CONNECTION RESTORED");
			}
			FlashUtil.delay(5);
		}
	}
	@Override
	protected void stopRobot(){
		stop = true;
		robotShutdown();
	}

	protected abstract void robotInit();
	protected abstract void robotShutdown();
	
	protected abstract void disabledInit();
	protected abstract void teleopInit();
	protected abstract void mappingInit();
	protected abstract void operationInit();
	
	protected abstract void disabledPeriodic();
	protected abstract void teleopPeriodic();
	protected abstract void mappingPeriodic();
	protected abstract void operationPeriodic();
}
