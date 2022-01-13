package frc.robot.team8583.lib.math.vectors;

import frc.robot.team254.lib.geometry.Translation2d;

public interface IVectorField {
	public abstract Translation2d getVector(Translation2d here);
}