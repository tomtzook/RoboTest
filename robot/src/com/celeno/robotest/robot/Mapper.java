package com.celeno.robotest.robot;

import java.util.Vector;

import edu.flash3388.flashlib.communications.Sendable;
import edu.flash3388.flashlib.math.Vector2;
import edu.flash3388.flashlib.util.FlashUtil;

public class Mapper extends Sendable{

	private static final byte NEW_DATA = 0x2;
	private static final byte DONE_AREA = 0x8;
	private static final byte ADVANCE_NEXT = 0x5;
	private static final byte START_MAPPING = 0xe;
	
	private Vector<Vector2> points = new Vector<Vector2>();
	private boolean sendDone = false;
	private boolean advanceNext = false;
	private boolean mappingStart = false;
	
	public Mapper() {
		super("mapper", RoboTestSendableType.MAPPER);
	}
	
	public void mapNewPoint(double x, double y){
		points.addElement(new Vector2(x, y));
	}
	public void reportMappingStart(){
		mappingStart = true;
	}
	public void reportDoneArea(){
		advanceNext = false;
		sendDone = true;
	}
	public boolean canAdvanceNext(){
		return advanceNext;
	}
	
	@Override
	public void newData(byte[] data) {
		if(data[0] == ADVANCE_NEXT){
			advanceNext = true;
		}
	}
	@Override
	public byte[] dataForTransmition() {
		if(mappingStart){
			mappingStart = false;
			return new byte[]{START_MAPPING};
		}
		if(points.size() > 1){
			Vector2 point = points.elementAt(0);
			points.remove(0);
			byte[] data = new byte[17];
			data[0] = NEW_DATA;
			FlashUtil.fillByteArray(point.getX(), 1, data);
			FlashUtil.fillByteArray(point.getY(), 9, data);
			return data;
		}
		if(sendDone){
			sendDone = false;
			return new byte[]{DONE_AREA};
		}
		return null;
	}
	@Override
	public boolean hasChanged() {
		return points.size() > 1 || sendDone || mappingStart;
	}

	@Override
	public void onConnection() {
	}
	@Override
	public void onConnectionLost() {
	}
}
