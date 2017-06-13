package com.celeno.robotest.robot;

import edu.flash3388.flashlib.math.Mathd;
import edu.flash3388.flashlib.robot.Action;
import edu.flash3388.flashlib.robot.InterruptionAction;
import edu.flash3388.flashlib.robot.PidController;
import edu.flash3388.flashlib.robot.systems.TankDriveSystem;
import edu.flash3388.flashlib.util.ConstantsHandler;

public class RobotMappingAction extends InterruptionAction{

	private Robot robot = Robot.getInstance();
	
	private Action alignAction, rotateAction, waitAdvanceOrder;
	
	private TankDriveSystem driveTrain;
	private PidController pidcontrollerDistance, pidControllerRotation;
	
	private boolean scanning = false;
	
	public RobotMappingAction() {
		driveTrain = robot.driveTrain;
		
		pidcontrollerDistance = new PidController(Robot.pid_kp.get(), Robot.pid_ki.get(), Robot.pid_kd.get(),
				()->0.0, robot.distancePidSource);
		pidControllerRotation = new PidController(Robot.pid_kp.get(), Robot.pid_ki.get(), Robot.pid_kd.get(), 
				()->0.0, robot.rotationPidSource);
	}
	
	
	private boolean frontBlocked(){
		return robot.sensorBase.isInfrontOfWall();
	}
	private boolean sideBlocked(){
		return robot.sensorBase.isRightBlocked();
	}
	private boolean sideAligned(){
		return robot.sensorBase.isLeftAligned();
	}
	
	private boolean atScanPoint(){
		return Robot.getInstance().getDistanceToTarget() <= Robot.d_margin_target.get();
	}
	private boolean isScanning(){
		return scanning;
	}
	private boolean scanEnded(){
		return robot.mapperSystem.isAreaDone();
	}
	private void performScan(){
		robot.mapperSystem.scanPoint();
	}
	private void setNewScanDest(){
		double anglerad = robot.sensorBase.getGyroAngleRad();
		
		ConstantsHandler.putNumber("destX", Robot.posX.get() + 
				Robot.mapping_distance.get() * Math.cos(anglerad));
		ConstantsHandler.putNumber("destY", Robot.posY.get() + 
				Robot.mapping_distance.get() * Math.sin(anglerad));
		
		robot.mapper.reportDoneArea();
	}
	private void setCurrentPointForScan(){
		ConstantsHandler.putNumber("destX", Robot.posX.get());
		ConstantsHandler.putNumber("destY", Robot.posY.get());
	}
	private void setScan(boolean scan){
		scanning = scan;
	}
	
	@Override
	protected void stdInitialize() {
		robot.mapper.reportMappingStart();
	}
	@Override
	protected void stdExecute() {
		if(isScanning() && atScanPoint()){
			performScan();
			if(scanEnded()){
				setNewScanDest();
				setScan(false);
				interrupt(waitAdvanceOrder);
			}
		}else{
			if(!sideAligned()){
				interrupt(alignAction);
				return;
			}
			if(frontBlocked() && !sideBlocked()){
				interrupt(rotateAction);
				setCurrentPointForScan();
				setScan(true);
				return;
			}
			if(atScanPoint()){
				setScan(true);
				return;
			}
			
			double speedY = Mathd.limit2(pidcontrollerDistance.calculate(), 
					Robot.minSpeed.get(), Robot.maxSpeed.get());
			double speedX = Mathd.limit2(pidControllerRotation.calculate(), 
					Robot.minSpeed.get(), Robot.maxSpeed.get());
			
			driveTrain.arcadeDrive(speedY, speedX);
		}
	}
	@Override
	protected boolean stdIsFinished() {
		return false;
	}

	@Override
	protected void stdEnd() {
	}
}
