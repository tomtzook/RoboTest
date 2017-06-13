package com.celeno.robotest.robot;

import edu.flash3388.flashlib.flashboard.DashboardInput;
import edu.flash3388.flashlib.flashboard.DoubleProperty;
import edu.flash3388.flashlib.flashboard.Flashboard;
import edu.flash3388.flashlib.flashboard.InputType;
import edu.flash3388.flashlib.math.Mathd;
import edu.flash3388.flashlib.robot.Action;
import edu.flash3388.flashlib.robot.FlashRoboUtil;
import edu.flash3388.flashlib.robot.InstantAction;
import edu.flash3388.flashlib.robot.PidController;
import edu.flash3388.flashlib.robot.PidSource;
import edu.flash3388.flashlib.robot.PidType;
import edu.flash3388.flashlib.robot.System;
import edu.flash3388.flashlib.robot.SystemAction;
import edu.flash3388.flashlib.robot.actions.PidDistanceActionPart;
import edu.flash3388.flashlib.robot.actions.PidRotationActionPart;
import edu.flash3388.flashlib.robot.devices.BooleanDataSource;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.robot.devices.FlashSpeedController;
import edu.flash3388.flashlib.robot.hid.XboxController;
import edu.flash3388.flashlib.robot.sbc.Servo;
import edu.flash3388.flashlib.robot.sbc.TalonSrx;
import edu.flash3388.flashlib.robot.systems.FlashDrive;
import edu.flash3388.flashlib.robot.systems.SingleMotorSystem;
import edu.flash3388.flashlib.util.ConstantsHandler;
import edu.flash3388.flashlib.util.Log;
import io.silverspoon.bulldog.beagleboneblack.BBBNames;

public class Robot extends RoboTestIterativeBot{

	private static Robot instance;
	
	Log robotLog;
	
	//hardware
	
	FlashSpeedController rightController;
	FlashSpeedController leftController;
	Servo mapperController; 
	
	//systems
	
	FlashDrive driveTrain;
	MappingSystem mapperSystem;
	SensorBase sensorBase;
	
	//control modules
	
	Mapper mapper;
	OperationController opController;
	PositionTracker posTracker;
	
	//hid
	
	XboxController xbox;
	
	//actions
	
	Action mappingAction;
	Action driveToTarget;
	Action teleopDrive;
	Action avoidCollision;
	
	final PidSource distancePidSource = new PidController.PidDoubleDataSource(()->getDistanceToTarget());
	final PidSource rotationPidSource = new PidController.PidDoubleDataSource(()->getAngleToTarget());
	
	//constants
	
	static final DoubleDataSource pid_kp = ConstantsHandler.addNumber("pid-kp", 1.0);
	static final DoubleDataSource pid_ki = ConstantsHandler.addNumber("pid-ki", 0.0);
	static final DoubleDataSource pid_kd = ConstantsHandler.addNumber("pid-kd", 0.0);
	static final DoubleDataSource d_margin_target = ConstantsHandler.addNumber("d-margin-target", 5.0);
	
	static final DoubleDataSource min_object_distance = ConstantsHandler.addNumber("min-object-distance", 80.0);
	static final DoubleDataSource mapping_distance = ConstantsHandler.addNumber("mapping-distance", 50.0);
	
	static final DoubleDataSource encoder_ticks_per_rev = ConstantsHandler.addNumber("ticks-per-rev", 1024.0);
	static final DoubleDataSource encoder_distance_per_rev = ConstantsHandler.addNumber("distance-per-rev", 10);
	
	static final DoubleDataSource robot_width = ConstantsHandler.addNumber("robot-width", 50.0);
	static final DoubleDataSource robot_length = ConstantsHandler.addNumber("robot-length", 50.0);
	
	static final DoubleDataSource minSpeed = ConstantsHandler.addNumber("min-speed", 0.1);
	static final DoubleDataSource maxSpeed = ConstantsHandler.addNumber("max-speed", 0.6);
	
	static final DoubleDataSource posX = ConstantsHandler.addNumber("posX", 0.0);
	static final DoubleDataSource posY = ConstantsHandler.addNumber("posY", 0.0);
	
	static final DoubleDataSource destX = ConstantsHandler.putNumber("destX", 0.0);
	static final DoubleDataSource destY = ConstantsHandler.putNumber("destY", 0.0);
	static final BooleanDataSource moveDest = ConstantsHandler.putBoolean("dest", false);
	
	//flashboard
	
	DoubleProperty posX_prop;
	DoubleProperty posY_prop;
	DoubleProperty destX_prop;
	DoubleProperty destY_prop;
	
	DashboardInput pid_kp_input;
	DashboardInput pid_ki_input;
	DashboardInput pid_kd_input;
	DashboardInput margin_target_input;
	DashboardInput maxSpeed_input;
	DashboardInput minSpeed_input;
	
	//--------------------------------------------------------------------
	//-------------------------Robot Init---------------------------------
	//--------------------------------------------------------------------
	
	@Override
	protected void robotInit() {
		instance = this;
		robotLog = new Log("robot");
		
		logRobot("Initializing systems and actions");
		validateSettings();
		initSystems();
		initActions();
		initHid();
		initSendables();
		initFlashboardData();
		logRobot("Initialization complete");
	}
	@Override
	protected void robotShutdown() {
		
	}
	
	private void validateSettings(){
		ConstantsHandler.addString("servo", BBBNames.PWM_P8_46);
		ConstantsHandler.addString("motor-left", BBBNames.PWM_P8_19);
		ConstantsHandler.addString("motor-right", BBBNames.PWM_P8_13);
		
		ConstantsHandler.addString("encoder-a", BBBNames.P8_10);
		ConstantsHandler.addString("encoder-b", BBBNames.P8_11);
		
		ConstantsHandler.addString("gyro-bus", BBBNames.I2C_0);
		
		ConstantsHandler.addNumber("sonic-count", 9);
		
		for (int i = 0; i < ConstantsHandler.getIntegerNative("sonic-count"); i++) {
			ConstantsHandler.addString("sonic"+i+"-trig", "");
			ConstantsHandler.addString("sonic"+i+"-echo", "");
		}
	}
	private void initSystems(){
		rightController = new TalonSrx(getPin(ConstantsHandler.getStringNative("motor-right")));
		leftController = new TalonSrx(getPin(ConstantsHandler.getStringNative("motor-left")));
		
		driveTrain = new FlashDrive(rightController, leftController);
		driveTrain.setName("DriveTrain");
		driveTrain.setMinSpeed(0.1);
		
		mapperController = new Servo(getPin(ConstantsHandler.getStringNative("servo")));
		mapperSystem = new MappingSystem(mapperController);
		mapperSystem.setName("MappingSystem");
		
		sensorBase = new SensorBase(posX, posY);
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
		teleopDrive.setName("TeleopDrive");
		
		PidDistanceActionPart distancePart = new TargetDistanceActionPart(pid_kp, pid_ki, pid_kd, d_margin_target);
		PidRotationActionPart rotationPart = new TargetRotationActionPart(pid_kp, pid_ki, pid_kd, d_margin_target);
		driveToTarget = new RobotCombinedAction(driveTrain, distancePart, rotationPart, minSpeed, maxSpeed);
		driveToTarget.setName("DriveToTarget");
		
		
		mappingAction = new RobotMappingAction(driveTrain, pid_kp, pid_ki, pid_kd, d_margin_target, minSpeed, maxSpeed);
		mappingAction.setName("MappingAction");
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
	private void initSendables(){
		mapper = new Mapper();
		opController = new OperationController();
		posTracker = new PositionTracker(posX, posY);
		
		getCommunications().attach(mapper, opController, posTracker);
	}
	private void initFlashboardData(){
		if(Flashboard.flashboardInit()){
			posX_prop = new DoubleProperty("posX", posX);
			posY_prop = new DoubleProperty("posY", posY);
			destX_prop = new DoubleProperty("destX", destX);
			destY_prop = new DoubleProperty("destY", destY);
			
			pid_kp_input = new DashboardInput("pid-kp", InputType.Double);
			pid_ki_input = new DashboardInput("pid-ki", InputType.Double);
			pid_kd_input = new DashboardInput("pid-kd", InputType.Double);
			margin_target_input = new DashboardInput("d-margin-target", InputType.Double);
			maxSpeed_input = new DashboardInput("min-speed", InputType.Double);
			minSpeed_input = new DashboardInput("max-speed", InputType.Double);
			
			
			Flashboard.attach(posX_prop, posY_prop, destX_prop, destY_prop, 
					pid_kp_input, pid_ki_input, pid_kd_input, margin_target_input, maxSpeed_input, minSpeed_input);
		}
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
		if(moveDest.get() && !driveToTarget.isRunning())
			driveToTarget.start();
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
	
	private void followWall(){
		
	}
	private void alignSide(){
		
	}
	private void scanPoint(){
		
	}
	
	//--------------------------------------------------------------------
	//-----------------------Robot External-------------------------------
	//--------------------------------------------------------------------
	
	public double getDistanceToTarget(){
		return Mathd.pythagorasTheorem(destX.get() - posX.get(), destY.get() - posY.get());
	}
	public double getAngleToTarget(){
		return Mathd.vecAzimuth(destY.get() - posY.get(), destX.get() - posX.get());
	}
	
	public double getDistanceTo(double x, double y){
		return Mathd.pythagorasTheorem(x - posX.get(), y - posY.get());
	}
	public double getAngleToTarget(double x, double y){
		return Mathd.vecAzimuth(x - posX.get(), y - posY.get());
	}
	
	public void logRobot(String data){
		robotLog.log(data, "Robot");
	}
	public void logRobot(String data, String caller){
		robotLog.log(data, caller);
	}
	
	//--------------------------------------------------------------------
	//------------------------Robot Static--------------------------------
	//--------------------------------------------------------------------
	
	public static Robot getInstance(){
		return instance;
	}
}
