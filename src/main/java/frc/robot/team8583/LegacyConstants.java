package frc.robot.team8583;

import java.util.Arrays;
import java.util.List;

import frc.robot.team8583.lib.util.InterpolatingDouble;
import frc.robot.team8583.lib.util.InterpolatingTreeMap;
import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;

@Deprecated
public class LegacyConstants
{
	/*All distance measurements are in inches, unless otherwise noted.*/

	public static final double kLooperDt = 0.02;

	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kDebuggingOutput = true;

	//CAN
	public static final int kCANTimeout = 16;

	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 36.5;
	public static final double kRobotLength = 36.5;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;
	public static final double kRobotProbeExtrusion = 4.0;

	public static final double kBallRadius = 6.5;

	//Field Landmarks
	public static final Pose2d kRobotLeftStartingPose = new Pose2d(
			new Translation2d(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotRightStartingPose = new Pose2d(
			new Translation2d(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotLeftRampExitPose = new Pose2d(
			new Translation2d(95.25 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotRightRampExitPose = new Pose2d(
			new Translation2d(95.25 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));

	//Swerve Calculations Constants (measurements are in inches)
	public static final double kWheelbaseLength = 289.675 / 25.4; // 289.675mm
	public static final double kWheelbaseWidth = 264.675 / 25.4; // 264.675mm
	public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);

	//Camera Constants
	public static final double kCameraYOffset = 0.25;
	public static final double kCameraXOffset = kRobotHalfLength - 15.0;
	public static final double kCameraZOffset = 16.45;
	public static final double kCameraYawAngleDegrees = 0.0;
	public static final double kCameraPitchAngleDegrees = kIsUsingCompBot ? 14.65 : 14.95;

	//Goal tracker constants
	public static double kMaxGoalTrackAge = 0.5;//0.5
	public static double kMaxTrackerDistance = 60.0;//18.0
	public static double kCameraFrameRate = 90.0;
	public static double kTrackReportComparatorStablityWeight = 1.0;
	public static double kTrackReportComparatorAgeWeight = 1.0;
	public static final double kDefaultCurveDistance = kRobotHalfLength + 36.0;
	public static final double kVisionUpdateDistance = kRobotHalfLength + 75.0;
	public static final double kVisionDistanceStep = 4.0;
	public static final double kClosestVisionDistance = 26.0;
	public static final double kDefaultVisionTrackingSpeed = 42.0;
	public static final double kCurvedVisionYOffset = 0.375;

	//Vision Speed Constraint Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();
	static
	{
		kVisionSpeedTreemap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(48.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(48.0));
	}

	//Path following constants
	public static final double kPathLookaheadTime = 0.25; // seconds to look ahead along the path for steering 0.4
	public static final double kPathMinLookaheadDistance = 6.0; // inches 24.0 (we've been using 3.0)

	//Swerve Speed Constants
	public static final double kSwerveRotationMaxSpeed = 14800.0 * 0.9375; // the 0.9375 is to request a speed that is always achievable
	public static final double kSwerveRotation10VoltMaxSpeed = 14800.0 * 0.9375;
	public static final double kSwerveRotationMaxAcceleration = kSwerveRotationMaxSpeed * 16.0;
	public static final double kSwerveDriveMaxSpeed = 28000.0;
	public static final double kSwerveDriveMaxAcceleration = kSwerveDriveMaxSpeed * 16.0;
	public static final double kSwerveMaxDriveSpeedInchesPerSecond = 12.5 * 12.0;

	//Swerve Power Contraints
	public static final double kSwerveRotationCurrentLimitTriggerCurrent = 30.0;
	public static final double kSwerveRotationMaxCurrent = kSwerveRotationCurrentLimitTriggerCurrent * 0.9375;
	public static final double kSwerveRotationCurrentLimitTriggeTime = 0.25;
	public static final double kSwerveDriveCurrentLimitTriggerCurrent = 40.0;
	public static final double kSwerveDriveMaxCurrent = kSwerveDriveCurrentLimitTriggerCurrent * 0.9375;
	public static final double kSwerveDriveCurrentLimitTriggeTime = 0.25;

	//Swerve Control Tolerance (Tolerated Closeloop Error)
	public static final double kSwerveRotationOnTargetTolerance = 360.0 / 256; // in degs
	public static final int kSwerveRotationControlTolerance = 0; // in enc units
	public static final int kSwerveDriveControlTolerance = 0; // in enc units

	//Swerve Odometry Constants
	public static final int kSwerveRotationEncoderResolution = 2048;
	public static final int kSwerveRotationCalibrationEncoderResolution = 4096;
	public static final double kSwerveRotationEncoderToWheelRatio = 1 / 12.0; // the number of rotations the swerve rotation encoder undergoes for every rotation of the wheel
	public static final double kSwerveRotationEncoderToExternalRotationEncoderRatio = kSwerveRotationCalibrationEncoderResolution
			/ kSwerveRotationEncoderResolution * kSwerveRotationEncoderToWheelRatio; // the number of enc units the swerve rotation encoder undergoes for every enc unit the swerve rotation calibration encoder undergoes
	public static final double kSwerveRotationEncUnitsPerRev = kSwerveRotationEncoderResolution
			/ kSwerveRotationEncoderToWheelRatio;
	public static final double kSwerveRotationEncUnitPerDegree = kSwerveRotationEncUnitsPerRev / 360.0;
	public static final double kSwerveWheelDiameter = 4.012; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901)
	public static final int kSwerveDriveEncoderResolution = 4096;
	public static final double kSwerveDriveEncoderToWheelRatio = 91.0 / 570.0; // the number of rotations the swerve drive encoder undergoes for every rotation of the wheel
	public static final double kSwerveDriveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution
			* kSwerveDriveEncoderToWheelRatio;
	public static final double kSwerveDriveEncUnitsPerInch = kSwerveDriveEncUnitsPerWheelRev
			/ (Math.PI * kSwerveWheelDiameter);

	//Swerve Module Wheel Rotation Calibration Offsets (Rotation calibration encoder values when the wheels are facing 0 degrees)
	public static final int kSwerveFrontRightRotationCalibrationOffset = 7776;
	public static final int kSwerveFrontLeftRotationCalibrationOffset = 17088;
	public static final int kSwerveRearLeftRotationCalibrationOffset = 11292;
	public static final int kSwerveRearRightRotationCalibrationOffset = 11220;

	//Swerve Module Positions (relative to the center of the drive base)
	public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength / 2,
			kWheelbaseWidth / 2);
	public static final Translation2d kVehicleToModuleOne = new Translation2d(-kWheelbaseLength / 2,
			kWheelbaseWidth / 2);
	public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength / 2,
			-kWheelbaseWidth / 2);
	public static final Translation2d kVehicleToModuleThree = new Translation2d(kWheelbaseLength / 2,
			-kWheelbaseWidth / 2);

	public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero, kVehicleToModuleOne,
			kVehicleToModuleTwo, kVehicleToModuleThree);

	//Swerve Speed Constraint Treemap
	public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSwerveSpeedTreeMap = new InterpolatingTreeMap<>();
	static
	{
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1.0));
	}

	//Wrist Constants
	public static final double kWristMaxSpeedHighGear = 98.7 * 2.0 * 4096.0 / 600.0;//encoder units per 100 ms
	public static final double kWristMaxSpeedLowGear = 31.32 * 4096.0 / 600.0;//200.0;
	public static final double kWristStartingAngle = 0.0;
	/** Pulse width position of the wrist encoder when the wrist is upright (at 90 degrees, parallel to the elevator). */
	public static final int kWristStartingEncoderPosition = kIsUsingCompBot ? 1239 : 1502;
	/** The number of rotations the wrist encoder undergoes for every rotation of the wrist. */
	public static final double kWristEncoderToOutputRatio = 30.0 / 12.0; // 144 degrees before wrap
	public static final double kWristAngleTolerance = 10.0; //degrees
	public static final double kWristMinControlAngle = -90.0; //degrees
	public static final double kWristMaxControlAngle = 85.0; //degrees
	public static final double kWristMinPhysicalAngle = -35.0;
	public static final double kWristMaxPhysicalAngle = 95.0;//95.192
	public static final double kWristIntakingAngle = kIsUsingCompBot ? 0.0 : 5.0;
	public static final double kWristPrimaryStowAngle = 60.5;
	public static final double kWristShortPlatformAngle = -32.0;
	public static final double kWristShortHangingAngle = -63.5;
	public static final double kWristHangingAngle = -70.5;//-71.5
	public static final double kWristBallHoldingAngle = 38.0;
	public static final double kWristBallFeedingAngle = 60.5;
	public static final double kWristMaxCurrent = 40.0;//amps

	//Ball Intake Constants
	public static final double kIntakeWeakEjectOutput = -0.75;
	public static final double kIntakeEjectOutput = kIsUsingCompBot ? -0.6 : -0.9;
	public static final double kIntakeStrongEjectOutput = -1.0;
	public static final double kIntakingOutput = 1.0;
	public static final double kIntakeWeakHoldingOutput = 2.0 / 12.0;
	public static final double kIntakeStrongHoldingOutput = 4.0 / 12.0;
	public static final double kIntakingResuckingOutput = 6.0 / 12.0;
	public static final double kIntakeRampRate = 0.25;
	public static final double kIntakeClimbOutput = 6.0 / 12.0;
	public static final double kIntakePullOutput = 12.0 / 12.0;
}
