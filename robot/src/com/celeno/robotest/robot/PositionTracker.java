package com.celeno.robotest.robot;

import edu.flash3388.flashlib.communications.Sendable;
import edu.flash3388.flashlib.robot.devices.DoubleDataSource;
import edu.flash3388.flashlib.util.ConstantsHandler;
import edu.flash3388.flashlib.util.FlashUtil;

public class PositionTracker extends Sendable{

	private static final byte POS_DATA = 0x5;
	private static final byte INIT_POS = 0xe;
	
	private DoubleDataSource x, y;
	private double lastX, lastY;
	
	public PositionTracker(DoubleDataSource x, DoubleDataSource y) {
		super("pos-tracker", RoboTestSendableType.POSITION_TRACKER);
		
		this.x = x;
		this.y = y;
	}

	@Override
	public void newData(byte[] data) {
		if(data[0] == INIT_POS){
			double x = FlashUtil.toDouble(data, 1);
			double y = FlashUtil.toDouble(data, 9);
			
			ConstantsHandler.putNumber("posX", x);
			ConstantsHandler.putNumber("posY", y);
		}
	}
	@Override
	public byte[] dataForTransmition() {
		lastX = x.get();
		lastY = y.get();
		
		byte[] data = new byte[17];
		data[0] = POS_DATA;
		FlashUtil.fillByteArray(lastX, 1, data);
		FlashUtil.fillByteArray(lastY, 9, data);
		return data;
	}
	@Override
	public boolean hasChanged() {
		return lastX != x.get() || lastY != y.get();
	}

	@Override
	public void onConnection() {
	}
	@Override
	public void onConnectionLost() {
	}
}
