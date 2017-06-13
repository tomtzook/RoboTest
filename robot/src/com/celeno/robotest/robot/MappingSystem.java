package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.sbc.Servo;
import edu.flash3388.flashlib.robot.systems.SingleMotorSystem;

public class MappingSystem extends SingleMotorSystem{

	private Robot robot = Robot.getInstance();
	private Servo servo;
	
	private double currentHeading = 0.0;
	
	public MappingSystem(Servo servo){
		super(servo);
		this.servo = servo;
	}
	
	public boolean isAreaDone(){
		return currentHeading >= Math.PI;
	}
	public void reset(){
		set(0.0);
		currentHeading = 0.0;
	}
	public void rotateScanner(){
		forward();
		currentHeading += 0.0;
	}
	public void scanPoint(){
		double distance = robot.sensorBase.distanceScanner();
		
		robot.mapper.mapNewPoint(
				Robot.posX.get() + distance * Math.cos(currentHeading), 
				Robot.posY.get() + distance * Math.sin(currentHeading)
		);
	}
}
