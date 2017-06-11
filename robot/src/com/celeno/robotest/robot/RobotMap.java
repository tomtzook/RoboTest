package com.celeno.robotest.robot;

import io.silverspoon.bulldog.beagleboneblack.BBBNames;

public class RobotMap {
	private RobotMap(){}
	
	public static String motor_right(){
		return BBBNames.P8_11;
	}
	public static String motor_left(){
		return BBBNames.P8_12;
	}
	
	public static String[][] sonic_pins(){
		return new String[][] {
			
		};
	}
	public static String gyro_bus(){
		return BBBNames.I2C_0;
	}
	public static String[] encoder_pins(){
		return new String[] {}; 
	}
}
