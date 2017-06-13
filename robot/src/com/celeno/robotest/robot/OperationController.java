package com.celeno.robotest.robot;

import edu.flash3388.flashlib.communications.Sendable;
import edu.flash3388.flashlib.util.ConstantsHandler;
import edu.flash3388.flashlib.util.FlashUtil;

public class OperationController extends Sendable{

	private static final byte OP_COMPLETED = 0x2;
	private static final byte OP_NEW = 0x5;
	private static final byte OP_START = 0x10;
	
	private boolean sendStartOp = false;
	private boolean sendOpCompleted = false;
	
	public OperationController() {
		super("op-controller", RoboTestSendableType.OPERATION_CONTROLLER);
	}

	public void reportOpCompleted(){
		sendOpCompleted = true;
		ConstantsHandler.putBoolean("dest", false);
	}
	
	@Override
	public void newData(byte[] data) {
		if(data[0] == OP_NEW){
			double x = FlashUtil.toDouble(data, 1);
			double y = FlashUtil.toDouble(data, 9);
			
			ConstantsHandler.putNumber("destX", x);
			ConstantsHandler.putNumber("destY", y);
			ConstantsHandler.putBoolean("dest", true);
			sendStartOp = true;
		}
		else if(data[0] == OP_COMPLETED){
			
		}
	}
	@Override
	public byte[] dataForTransmition() {
		if(sendOpCompleted){
			sendOpCompleted = false;
			return new byte[] {OP_COMPLETED};
		}
		if (sendStartOp) {
			sendStartOp = false;
			return new byte[] {OP_START};
		}
		return null;
	}
	@Override
	public boolean hasChanged() {
		return sendOpCompleted || sendStartOp;
	}

	@Override
	public void onConnection() {
	}
	@Override
	public void onConnectionLost() {
	}
}
