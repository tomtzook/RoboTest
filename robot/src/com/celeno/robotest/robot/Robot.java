package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.Action;
import edu.flash3388.flashlib.robot.FlashRoboUtil;
import edu.flash3388.flashlib.robot.InstantAction;
import edu.flash3388.flashlib.robot.System;
import edu.flash3388.flashlib.robot.SystemAction;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.robot.devices.FlashSpeedController;
import edu.flash3388.flashlib.robot.hid.XboxController;
import edu.flash3388.flashlib.robot.sbc.TalonSrx;
import edu.flash3388.flashlib.robot.systems.FlashDrive;
import edu.flash3388.flashlib.util.ConstantsHandler;
import edu.flash3388.flashlib.util.Log;

public class Robot extends RoboTestIterativeBot{

	static Robot instance;
	
	Log robotLog;
	
	FlashSpeedController rightController;
	FlashSpeedController leftController;
	
	FlashDrive driveTrain;
	SensorBase sensorBase;
	
	XboxController xbox;
	
	Action mappingAction;
	Action driveToTarget;
	Action teleopDrive;
	
	DoubleDataSource posX = ConstantsHandler.addNumber("posX", 0.0);
	DoubleDataSource posY = ConstantsHandler.addNumber("posY", 0.0);
	
	//--------------------------------------------------------------------
	//-------------------------Robot Init---------------------------------
	//--------------------------------------------------------------------
	
	@Override
	protected void robotInit() {
		robotLog = new Log("robot");
		
		logRobot("Initializing systems and actions");
		initSystems();
		initActions();
		initHid();
		logRobot("Initialization complete");
	}
	@Override
	protected void robotShutdown() {
		
	}
	
	private void initSystems(){
		rightController = new TalonSrx(getPin(RobotMap.motor_right()));
		leftController = new TalonSrx(getPin(RobotMap.motor_left()));
		
		driveTrain = new FlashDrive(rightController, leftController);
		driveTrain.setName("DriveTrain");
		driveTrain.setMinSpeed(0.1);
		
		sensorBase = new SensorBase();
	}
	private void initActions(){
		teleopDrive = new SystemAction(driveTrain, new Action(){
			@Override
			protected void execute() {
				driveTrain.arcadeDrive(xbox.LeftStick, xbox.RightStick);
			}
			@Override
			protected void end() {
				driveTrain.stop();
			}
		});
	}
	private void initHid(){
		InstantAction stop = new InstantAction(){
			@Override
			protected void execute() {
				 stopAll();
			}
		};
		
		xbox = new XboxController(0);
		xbox.A.whenPressed(stop);
	}
	
	//--------------------------------------------------------------------
	//-----------------------Robot States---------------------------------
	//--------------------------------------------------------------------
	
	@Override
	protected void disabledInit() {
		stopAll();
	}
	@Override
	protected void disabledPeriodic() {
		sensorBase.update();
		
	}
	
	@Override
	protected void teleopInit() {
		stopAll();
		teleopDrive.start();
	}
	@Override
	protected void teleopPeriodic() {
		FlashRoboUtil.updateHID();
		sensorBase.update();
	}

	@Override
	protected void mappingInit() {
		stopAll();
	}
	@Override
	protected void mappingPeriodic() {
		sensorBase.update();
	}

	@Override
	protected void operationInit() {
		stopAll();
	}
	@Override
	protected void operationPeriodic() {
		sensorBase.update();
	}
	
	//--------------------------------------------------------------------
	//-----------------------Robot General--------------------------------
	//--------------------------------------------------------------------
	
	private void stopAll(){
		logRobot("Stopping all");
		
		 cancelSystemAction(driveTrain);
		 
		 driveTrain.stop();
	}
	private void cancelSystemAction(System system){
		Action a = system.getCurrentAction();
		if(a != null && a.isRunning()){
			a.cancel();
			logRobot("Canceling action: "+a.getName(), system.getName());
		}
	}
	private void cancelAction(Action a){
		if(a != null && a.isRunning()){
			a.cancel();
			logRobot("Canceling action: "+a.getName());
		}
	}
	
	private void logRobot(String data){
		robotLog.log(data, "Robot");
	}
	private void logRobot(String data, String caller){
		robotLog.log(data, caller);
	}
	
	//--------------------------------------------------------------------
	//-----------------------Robot External-------------------------------
	//--------------------------------------------------------------------
	
	public SensorBase getSensorBase(){
		return sensorBase;
	}
	
	//--------------------------------------------------------------------
	//------------------------Robot Static--------------------------------
	//--------------------------------------------------------------------
	
	public static Robot getInstance(){
		return instance;
	}
}
