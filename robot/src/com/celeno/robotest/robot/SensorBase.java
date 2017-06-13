package com.celeno.robotest.robot;

import edu.flash3388.flashlib.math.Mathd;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.robot.devices.Gyro;
import edu.flash3388.flashlib.robot.devices.RangeFinder;
import edu.flash3388.flashlib.robot.sbc.QuadEncoder;
import edu.flash3388.flashlib.robot.sbc.Ultrasonic;
import edu.flash3388.flashlib.util.ConstantsHandler;

public class SensorBase {

	public static final int SONIC_RIGHT = 0;
	public static final int SONIC_LEFT = 1;
	
	private DoubleDataSource posX, posY;
	
	private RangeFinder[] sonics;
	private DoubleDataSource.VarDataSource[] sonicProps;
	
	private QuadEncoder encoder;
	private DoubleDataSource.VarDataSource encoderProp;
	
	private Gyro gyro;
	private DoubleDataSource.VarDataSource gyroProp;
	
	private double distance, angle;
	
	public SensorBase(DoubleDataSource posX, DoubleDataSource posY){
		this.posX = posX;
		this.posY = posY;
		
		String trig = null;
		String echo = null;
		
		int sonicCount = ConstantsHandler.getIntegerNative("sonic-count");
		sonics = new RangeFinder[sonicCount];
		sonicProps = new DoubleDataSource.VarDataSource[sonicCount];
		for (int i = 0; i < sonicCount; i++) {
			sonicProps[i] = new DoubleDataSource.VarDataSource();
			
			trig = ConstantsHandler.getStringNative("sonic"+i+"-trig");
			echo = ConstantsHandler.getStringNative("sonic"+i+"-echo");
			
			sonics[i] = new Ultrasonic(Robot.getPin(trig), Robot.getPin(echo));
			
			if(trig.isEmpty() || echo.isEmpty())
				throw new IllegalArgumentException("Sonic "+i+" is missing a pin name");
		}
		
		String encoderPinA = ConstantsHandler.getStringNative("encoder-a");
		String encoderPinB = ConstantsHandler.getStringNative("encoder-b");
		encoderProp = new DoubleDataSource.VarDataSource();
		encoder = new QuadEncoder(Robot.getPin(encoderPinA), Robot.getPin(encoderPinB));
		encoder.setDistancePerRevolution(Robot.encoder_distance_per_rev.get());
		encoder.setTicksPerRevolution((int)Robot.encoder_ticks_per_rev.get());
		
		String gyroBus = ConstantsHandler.getStringNative("gyro-bus");
		gyroProp = new DoubleDataSource.VarDataSource();
	}
	
	public double getEncoderDistanceRaw(){
		return encoder.getDistance();
	}
	public double getGyroAngleRaw(){
		return 0.0;
	}
	public double getGyroAngleRadRaw(){
		return Math.toRadians(getGyroAngleRaw());
	}
	public double getUltrasonicRangeRaw(int sonic){
		return sonics[sonic].getRangeCM();
	}
	
	public void resetEncoder(){
		encoder.reset();
	}
	
	public DoubleDataSource getEncoderDistanceProperty(){
		return encoderProp;
	}
	public DoubleDataSource getGyroAngleProperty(){
		return gyroProp;
	}
	public DoubleDataSource getUltrasonicRangeProperty(int sonic){
		return sonicProps[sonic];
	}
	
	public double getEncoderDistance(){
		return encoderProp.get();
	}
	public double getGyroAngle(){
		return gyroProp.get();
	}
	public double getGyroAngleRad(){
		return Math.toRadians(getGyroAngle());
	}
	public double getUltrasonicRange(int sonic){
		return sonicProps[sonic].get();
	}
	
	public double distanceScanner(){
		return getUltrasonicRange(8);
	}
	public double distanceFront(){
		return Mathd.avg(getUltrasonicRange(0), getUltrasonicRange(1));
	}
	public double distanceFront(int sonic){
		return getUltrasonicRange(sonic);
	}
	public double distanceBack(){
		return Mathd.avg(getUltrasonicRange(2), getUltrasonicRange(3));
	}
	public double distanceBack(int sonic){
		return getUltrasonicRange(2 + sonic);
	}
	public double distanceLeft(){
		return Mathd.avg(getUltrasonicRange(4), getUltrasonicRange(5));
	}
	public double distanceLeft(int sonic){
		return getUltrasonicRange(4 + sonic);
	}
	public double distanceRight(){
		return Mathd.avg(getUltrasonicRange(6), getUltrasonicRange(7));
	}
	public double distanceRight(int sonic){
		return getUltrasonicRange(6 + sonic);
	}
	
	public void update(){
		for (int i = 0; i < sonicProps.length; i++)
			sonicProps[i].set(getUltrasonicRangeRaw(i));
		
		distance = getEncoderDistanceRaw();
		resetEncoder();
		
		angle = getGyroAngleRaw();
		
		gyroProp.set(angle);
		encoderProp.set(distance);
		
		double anglerad = Math.toRadians(angle);
		
		ConstantsHandler.putNumber("posX", posX.get() + distance * Math.cos(anglerad));
		ConstantsHandler.putNumber("posY", posY.get() + distance * Math.sin(anglerad));
	}
	
	public boolean isInfrontOfWall(){
		return distanceFront() <= Robot.min_object_distance.get() + Robot.d_margin_target.get();
	}
	public boolean isFrontAligned(){
		return Math.abs(distanceFront(0) - distanceFront(1)) <= 5.0;
	}
	public boolean isRightBlocked(){
		return distanceRight() <= Robot.min_object_distance.get() + Robot.d_margin_target.get();
	}
	public boolean isRightAligned(){
		return Math.abs(distanceRight(0) - distanceRight(1)) <= 5.0;
	}
	public boolean isLeftBlocked(){
		return distanceLeft() <= Robot.min_object_distance.get() + Robot.d_margin_target.get();
	}
	public boolean isLeftAligned(){
		return Math.abs(distanceLeft(0) - distanceLeft(1)) <= 5.0;
	}
}
