package com.celeno.robotest.robot;

import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.robot.devices.Encoder;
import edu.flash3388.flashlib.robot.devices.Gyro;
import edu.flash3388.flashlib.robot.devices.RangeFinder;

public class SensorBase {

	private RangeFinder[] sonics;
	private DoubleDataSource.VarDataSource[] sonicProps;
	
	private Encoder encoder;
	private DoubleDataSource.VarDataSource encoderProp;
	
	private Gyro gyro;
	private DoubleDataSource.VarDataSource gyroProp;
	
	public SensorBase(){
		String[][] sonicPins = RobotMap.sonic_pins();
		sonics = new RangeFinder[sonicPins.length];
		sonicProps = new DoubleDataSource.VarDataSource[sonicPins.length];
		for (int i = 0; i < sonicPins.length; i++) {
			sonicProps[i] = new DoubleDataSource.VarDataSource();
		}
		
		String[] encoderPins = RobotMap.encoder_pins();
		encoderProp = new DoubleDataSource.VarDataSource();
		
		String gyroBus = RobotMap.gyro_bus();
		gyroProp = new DoubleDataSource.VarDataSource();
	}
	
	public double getEncoderRateRaw(){
		return 0.0;
	}
	public double getGyroAngleRaw(){
		return 0.0;
	}
	public double getUltrasonicRangeRaw(int sonic){
		return 0.0;
	}
	
	public DoubleDataSource getEncoderRateProperty(){
		return encoderProp;
	}
	public DoubleDataSource getGyroAngleProperty(){
		return gyroProp;
	}
	public DoubleDataSource getUltrasonicRangeProperty(int sonic){
		return sonicProps[sonic];
	}
	
	public double getEncoderRate(){
		return encoderProp.get();
	}
	public double getGyroAngle(){
		return gyroProp.get();
	}
	public double getUltrasonicRange(int sonic){
		return sonicProps[sonic].get();
	}
	
	public void update(){
		for (int i = 0; i < sonicProps.length; i++) 
			sonicProps[i].set(getUltrasonicRangeRaw(i));
		gyroProp.set(getGyroAngleRaw());
		encoderProp.set(getEncoderRateRaw());
	}
}
