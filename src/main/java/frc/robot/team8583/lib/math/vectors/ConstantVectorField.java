package frc.robot.team8583.lib.math.vectors;

import frc.robot.team254.lib.geometry.Translation2d;


public class ConstantVectorField extends VectorField {
	public ConstantVectorField(Translation2d whichWay) {
		thatWay = whichWay.normalize();
	}
	
	protected Translation2d thatWay;
	
	public Translation2d getVector(Translation2d here) {
		return thatWay;
	}
}