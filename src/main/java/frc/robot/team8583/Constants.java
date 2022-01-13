package frc.robot.team8583;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.lib.util.Util;

public class Constants
{
	// All distance measurements are in seconds, inches, degrees, volts, and amps, unless otherwise noted

	public static class Looper
	{
		public static final double DELTA_TIME = 0.02;
		public static final int DELTA_TIME_MS = Util.roundToInt(DELTA_TIME * 1000.0);
	}

	public static class Log
	{
		public static final boolean ENABLE_DEBUG_OUTPUT = true;
	}

	public static class Can
	{
		public static final int TIMEOUT = Looper.DELTA_TIME_MS;
		public static final double MOTOR_CONTROLLER_CONFIG_DELAY = 0.75;
	}

	public static class Physics
	{
		public static final double G = 9806.65 / 25.4;
		public static final double kEpsilon = Math.ulp(Math.pow(2.0, 16.0));
	}

	public static class Robot
	{
		// Physical Dimensions (including bumpers)
		public static final double WIDTH = 0.0;
		public static final double LENGTH = 0.0;
		public static final double HALF_WIDTH = WIDTH / 2.0;
		public static final double HALF_LENGTH = LENGTH / 2.0;

		// Initial States
		public static final double INIT_HEADING = 180.0;
	}

	public static class Field
	{
		// Physical Dimensions of Field Objects
		public static final double BALL_RADIUS = 3.5;
		public static final double VISUAL_TARGET_CENTER_HEIGHT = 98.25;
		public static final double VISUAL_TARGET_VISUAL_CENTER_HEIGHT = 89.75;
		public static final double VISUAL_TARGET_DEPTH = 29.25;

		//Auto Poses

		//Left
		public static final Pose2d STARTING_POSE_LEFT = new Pose2d(new Translation2d(-18.5, 297.32),
				Rotation2d.fromDegrees(180.0));

		//Right
		/*public static final Pose2d STARTING_POSE_RIGHT = new Pose2d(new Translation2d(-18.5, 27.5),
				Rotation2d.fromDegrees(180.0));
		
		public static final Pose2d RIGHT_TRENCH_END_POINT = new Pose2d(new Translation2d(-180, 27.5),
				Rotation2d.fromDegrees(180.0));
		
		public static final Pose2d RIGHT_CENTRAL_3_BALL_START_POINT_1 = new Pose2d(new Translation2d(-200, 106.66),
				Rotation2d.fromDegrees(180.0));
		
		public static final Pose2d RIGHT_CENTRAL_3_BALL_START_POINT_2 = new Pose2d(new Translation2d(-200, 106.66),
				Rotation2d.fromDegrees(30.0));
		
		public static final Pose2d RIGHT_CENTRAL_3_BALL_END_POINT = new Pose2d(new Translation2d(-140, 132.65),
				Rotation2d.fromDegrees(30.0));
		
		public static final Pose2d RIGHT_CENTRAL_2_BALL_START_POINT = new Pose2d(new Translation2d(-200, 116.66),
				Rotation2d.fromDegrees(30.0));
		
		public static final Pose2d RIGHT_CENTRAL_2_BALL_WAY_POINT = new Pose2d(new Translation2d(-190.62, 150.66),
				Rotation2d.fromDegrees(30.0));
		
		public static final Pose2d RIGHT_CENTRAL_2_BALL_END_POINT = new Pose2d(new Translation2d(-131.91, 166.93),
				Rotation2d.fromDegrees(30.0));
		
		public static final Pose2d RIGHT_SHOOT_WAY_POINT = new Pose2d(new Translation2d(-131.91, 90.66),
				Rotation2d.fromDegrees(30.0));
		
		public static final Pose2d RIGHT_SHOOT_END_POINT = new Pose2d(new Translation2d(-100, 90.66),
				Rotation2d.fromDegrees(30.0));*/

		public static final Pose2d STARTING_POSE_RIGHT = new Pose2d(new Translation2d(-19.5, 20.2),
				Rotation2d.fromDegrees(180.0));
		public static final Pose2d RIGHT_TRENCH_WAT_POINT_1 = new Pose2d(new Translation2d(-50.823, 24.631),
				Rotation2d.fromDegrees(180.0));
		public static final Pose2d RIGHT_TRENCH_WAT_POINT_2 = new Pose2d(new Translation2d(-88.468, 27.653),
				Rotation2d.fromDegrees(180.0));
		public static final Pose2d RIGHT_TRENCH_WAT_POINT_3 = new Pose2d(new Translation2d(-132.292, 29.312),
				Rotation2d.fromDegrees(180.0));
		public static final Pose2d RIGHT_TRENCH_END_POINT = new Pose2d(new Translation2d(-182.293, 29.608),
				Rotation2d.fromDegrees(180.0));

		public static final Pose2d RIGHT_CENTRAL_3_BALL_START_POINT = new Pose2d(new Translation2d(-182.293, 29.608),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_1 = new Pose2d(new Translation2d(-188.723, 49.962),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_2 = new Pose2d(new Translation2d(-193.673, 66.484),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_3 = new Pose2d(new Translation2d(-196.732, 79.77),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_4 = new Pose2d(new Translation2d(-197.492, 90.416),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_5 = new Pose2d(new Translation2d(-195.545, 99.019),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_6 = new Pose2d(new Translation2d(-190.481, 106.173),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_7 = new Pose2d(new Translation2d(-181.891, 112.475),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_8 = new Pose2d(new Translation2d(-169.366, 118.521),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_WAY_POINT_9 = new Pose2d(new Translation2d(-152.497, 124.907),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_3_BALL_END_POINT = new Pose2d(new Translation2d(-130.876, 132.228),
				Rotation2d.fromDegrees(30.0));

		public static final Pose2d RIGHT_CENTRAL_RETREAT_WAY_POINT_1 = new Pose2d(new Translation2d(-151.208, 125.777),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_RETREAT_WAY_POINT_2 = new Pose2d(new Translation2d(-166.369, 126.44),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_RETREAT_WAY_POINT_3 = new Pose2d(new Translation2d(-176.358, 134.217),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_CENTRAL_2_BALL_START_POINT = new Pose2d(new Translation2d(-181.177, 149.108),
				Rotation2d.fromDegrees(30.0));

		public static final Pose2d RIGHT_SHOOT_WAY_POINT_1 = new Pose2d(new Translation2d(-148.033, 125.715),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_2 = new Pose2d(new Translation2d(-160.871, 122.751),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_3 = new Pose2d(new Translation2d(-169.806, 122.773),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_4 = new Pose2d(new Translation2d(-175.254, 125.216),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_5 = new Pose2d(new Translation2d(-177.631, 129.517),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_6 = new Pose2d(new Translation2d(-177.356, 135.112),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_7 = new Pose2d(new Translation2d(-174.843, 141.436),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_8 = new Pose2d(new Translation2d(-170.509, 147.927),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_9 = new Pose2d(new Translation2d(-164.771, 154.019),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_10 = new Pose2d(new Translation2d(-158.045, 159.15),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_11 = new Pose2d(new Translation2d(-150.748, 162.755),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_12 = new Pose2d(new Translation2d(-143.295, 164.269),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_13 = new Pose2d(new Translation2d(-136.105, 163.131),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_14 = new Pose2d(new Translation2d(-129.592, 158.774),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_15 = new Pose2d(new Translation2d(-124.174, 150.636),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_16 = new Pose2d(new Translation2d(-120.267, 138.153),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_WAY_POINT_17 = new Pose2d(new Translation2d(-118.287, 120.76),
				Rotation2d.fromDegrees(30.0));
		public static final Pose2d RIGHT_SHOOT_END_POINT = new Pose2d(new Translation2d(-118.651, 97.894),
				Rotation2d.fromDegrees(30.0));

	}

	public static class DriverInterface
	{
		// Controller Ports
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;

		// Button Thresholds
		public static final double TRIGGER_PRESS_THRESHOLD = 0.5;
	}

	public static class Ahrs
	{
		// Initialization Parameters
		public static final double INIT_TIMEOUT = 4.0;
		public static final double INIT_DELAY = 0.25;

		// Communication Parameters
		public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;

		// Compensation Parameters
		public static final boolean TEMPERATURE_COMPENSATION = true;
	}

	public static class VideoFeed
	{
		// Video Parameters
		public static final int RESOLUTION_WIDTH = 480;
		public static final int RESOLUTION_HEIGHT = 240;
		public static final int FRAMERATE = 15;
	}

	public static class Swerve
	{
		// Mechanical Constants
		public static final double WHEELBASE_HALF_LENGTH = 559.35 / 2.0 / 25.4;
		public static final double WHEELBASE_HALF_WIDTH = 559.35 / 2.0 / 25.4;
		public static final double WHEELBASE_HALF_DIAGONAL = Math.hypot(WHEELBASE_HALF_LENGTH, WHEELBASE_HALF_WIDTH);
		public static final int MODULE_COUNT = 4;

		// Manual Operation Parameters
		public static final double TRANSLATIONAL_SPEED_FACTOR = 0.75;
		public static final double ROTATIONAL_SPEED_FACTOR = 0.5;
		public static final double SLOW_MODE_TRANSLATIONAL_SPEED_ATTENUATION_FACTOR = 0.5;
		public static final double SLOW_MODE_ROTATIONAL_SPEED_ATTENUATION_FACTOR = 0.0;
		public static final double TRANSLATION_INPUT_CIRCULAR_DEADBAND = 0.09375;
		public static final double ROTATION_INPUT_DEADBAND = 0.09375;
		public static final boolean ENABLE_HEADING_STABILIZATION = true;
		public static final double HEADING_STABILIZATION_ANGULAR_VELOCITY_THRESHOLD = 360.0 / 64.0;

		public static class DriveModule
		{
			// Module Relative Positions (relative to the center of the drive base)
			public static final Translation2d FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER = new Translation2d(
					WHEELBASE_HALF_LENGTH, WHEELBASE_HALF_WIDTH);
			public static final Translation2d REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER = new Translation2d(
					-WHEELBASE_HALF_LENGTH, WHEELBASE_HALF_WIDTH);
			public static final Translation2d REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER = new Translation2d(
					-WHEELBASE_HALF_LENGTH, -WHEELBASE_HALF_WIDTH);
			public static final Translation2d FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER = new Translation2d(
					WHEELBASE_HALF_LENGTH, -WHEELBASE_HALF_WIDTH);
			public static final List<Translation2d> POSITIONS_RELATIVE_TO_DRIVE_CENTER = Arrays.asList(
					FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
					REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
					REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
					FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);

			public static class Translation
			{
				// Control Inversion
				public static final boolean INVERT_MOTOR = true;

				// Speed Constants
				public static final double SPEED_RESERVE_FACTOR = 1.0 / 16.0;
				public static final double MAX_SPEED = 18600.0; // in encoder units / 0.1s
				public static final double MAX_CRUISE_SPEED = MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
				public static final VelocityMeasPeriod SPEED_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_1Ms;
				public static final int SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;

				// Power Constraints
				public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
				public static final double SATURATION_VOLTAGE = 10.0;
				public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
				public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
				public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 48.0;
				public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
						* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
				public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

				// Error Tolerance
				public static final int CONTROL_ERROR_TOLERANCE = 0; // in encoder units
				public static final double SPEED_ON_TARGET_ERROR_TOLERANCE = 1.0;
				public static final double DISTANCE_ON_TARGET_ERROR_TOLERANCE = 1.0;

				// PID Parameters
				// Slot 0 is for velocity mode
				public static final double PID0_KP = 0.0625;
				public static final double PID0_KI = 0.0;
				public static final double PID0_KD = 0.0;
				public static final double PID0_KF = 1023.0 / MAX_SPEED;
				// Slot 1 for Position Mode (Motion Magic)
				public static final double PID1_KP = 0.0;
				public static final double PID1_KI = 0.0;
				public static final double PID1_KD = 0.0;
				public static final double PID1_KF = 1023.0 / MAX_SPEED;

				// Communication Parameters
				public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;

				// Mechanical Constants
				public static final double WHEEL_DIAMETER = 4.012;
				public static final int ENCODER_RESOLUTION = 2048;
				public static final double ENCODER_TO_WHEEL_RATIO = 570.0 / 91.0; // the number of rotations the encoder undergoes for every rotation of the wheel
				public static final double ENCODER_UNITS_PER_WHEEL_REVOLUTION = ENCODER_RESOLUTION
						* ENCODER_TO_WHEEL_RATIO;
				public static final double ODOMETER_ERROR_COMPENSATION_FACTOR = 1.0;
				public static final double ENCODER_UNITS_PER_INCH = ENCODER_UNITS_PER_WHEEL_REVOLUTION
						/ (Math.PI * WHEEL_DIAMETER) * ODOMETER_ERROR_COMPENSATION_FACTOR;
			}

			public static class Rotation
			{
				// Control Inversion
				public static final boolean INVERT_MOTOR = true;
				public static final boolean INVERT_EXTERNAL_SENSOR = true;

				// Speed Constants
				public static final double SPEED_RESERVE_FACTOR = 1.0 / 16.0;
				public static final double MAX_SPEED = 14600.0; // in encoder units / 0.1s
				public static final double MAX_CRUISE_SPEED = MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
				public static final double MAX_ACCELERATION = MAX_SPEED * 16.0; // estimated, in encoder units / 0.1s^2
				public static final VelocityMeasPeriod SPEED_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_1Ms;
				public static final int SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;

				// Power Constraints
				public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
				public static final double SATURATION_VOLTAGE = 8.0;
				public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
				public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
				public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 32.0;
				public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
						* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
				public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

				// Error Tolerance
				public static final int CONTROL_ERROR_TOLERANCE = 0; // in encoder units
				public static final double HEADING_ON_TARGET_ERROR_TOLERANCE = 360.0 / 256.0;

				// PID Parameters
				public static final double PID0_KP = 2.0;
				public static final double PID0_KI = 0.0;
				public static final double PID0_KD = 24.0;
				public static final double PID0_KF = 1023.0 / MAX_SPEED;

				// Mechanical Constants
				public static final int ENCODER_RESOLUTION = 2048;
				public static final int CALIBRATION_ENCODER_RESOLUTION = 4096;
				public static final double ENCODER_TO_MODULE_BASE_RATIO = 12.0; // the number of rotations the encoder undergoes for every rotation of the base of the module
				public static final double ENCODER_TO_EXTERNAL_ENCODER_RATIO = CALIBRATION_ENCODER_RESOLUTION
						/ ENCODER_RESOLUTION / ENCODER_TO_MODULE_BASE_RATIO; // the number of encoder units the encoder undergoes for every encoder unit the calibration encoder undergoes
				public static final double ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION = ENCODER_RESOLUTION
						* ENCODER_TO_MODULE_BASE_RATIO;
				public static final double ENCODER_UNITS_PER_DEGREE = ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION / 360.0;

				// Communication Parameters
				public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;
				public static final double CALIBRATION_DELAY = 0.25;

				// Calibration Offsets (calibration encoder values when the wheels are facing 0 degrees)
				public static final int FRONT_LEFT_CALIBRATION_OFFSET = 2358;
				public static final int REAR_LEFT_CALIBRATION_OFFSET = 5251;
				public static final int REAR_RIGHT_CALIBRATION_OFFSET = 18312;
				public static final int FRONT_RIGHT_CALIBRATION_OFFSET = 12378;
			}
		}

		public static class Odometer
		{
			public static final double DISTANCE_DEVIANCE_THRESHOLD = 1.0;
		}

		public static class HeadingController
		{
			public class Manual
			{
				// PID Parameters
				public static final double KP = 0.5;
				public static final double KI = 0.0;
				public static final double KD = 0.0;
				public static final double CONTROL_ERROR_TOLERANCE = 0.0;
			}

			public class Auto
			{
				// PID Parameters
				public static final double KP = 0.5;
				public static final double KI = 0.0;
				public static final double KD = 0.0;
				public static final double CONTROL_ERROR_TOLERANCE = 0.0;
			}

		}

		public static class TranslationController
		{
			public static class X
			{
				// PID Parameters
				public static final double KP = 1.0;
				public static final double KI = 0.0;
				public static final double KD = 0.0;
				public static final double CONTROL_ERROR_TOLERANCE = 0.0;
			}

			public static class Y
			{
				// PID Parameters
				public static final double KP = 1.0;
				public static final double KI = 0.0;
				public static final double KD = 0.0;
				public static final double CONTROL_ERROR_TOLERANCE = 0.0;
			}

		}

		public static class DriveMotionPlanner
		{
			//Path following constants
			public static final double kPathLookaheadTime = 0.25; // seconds to look ahead along the path for steering 0.4
			public static final double kPathMinLookaheadDistance = 3.0; // inches 24.0 (we've been using 3.0)

			public static final double kSwerveMaxDriveSpeedInchesPerSecond = 12.5 * 10.0
					* Swerve.DriveModule.Translation.MAX_CRUISE_SPEED
					/ Swerve.DriveModule.Translation.ENCODER_UNITS_PER_WHEEL_REVOLUTION;

			public static final double kRotationFactor = 3.0;
			public static final double kMinTranslationVelocity = 5.0;
		}

		public static class TrajectoryGenerator
		{
			public static final double kMaxVelocity = 100.0;
			public static final double kMaxAccel = 100.0;
			public static final double kMaxDecel = 80.0;
			public static final double kMaxVoltage = 9.0;

			public static final double kMaxDx = 2.0;
			public static final double kMaxDy = 2.0;
			public static final double kMaxDTheta = Math.toRadians(5.0);
		}
	}

	public static class Shooter
	{
		public static class Turret
		{
			// Targeting Parameters
			public static final boolean APPLY_DEPTH_COMPENSATION = false;
			public static final double LOSE_TARGET_TIME_THRESHOLD = 1.0;
			public static final int ROTATION_STATE_HISTORY_MAX_LENGTH = 16;
			public static final boolean APPLY_CUBIC_INTERPOLATION = true;
			public static final double AVERAGE_DRAG_INDUCED_X_ACCELERATION = 32.0;
			public static final double AVERAGE_DRAG_INDUCED_Y_ACCELERATION = 24.0;

			public static class Vision
			{
				// Video Characteristics
				public static final double FRAMERATE = 22.0;
				public static final double LATENCY_OFFSET = 0.0;

				// Mechanical Constants
				public static final double CAMERA_HEIGHT = 515.91 / 25.4;
				public static final double CAMERA_ANGLE_OF_ELEVATION = 90.0 - 65.0;
				public static final double CAMERA_HORIZONTAL_ANGULAR_OFFSET = -2.8125;
			}

			public static class YawGimbal
			{
				// Control Inversion
				public static final boolean INVERT_MOTOR = false;

				// Speed Constants
				public static final double SPEED_RESERVE_FACTOR = 1.0 / 16.0;
				public static final double MAX_SPEED = 14400.0; // in encoder units / 0.1s
				public static final double MAX_CRUISE_SPEED = MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
				public static final double MAX_ACCELERATION = MAX_SPEED * 16.0; // estimated, in encoder units / 0.1s^2
				public static final int CURVE_STRENGTH = 4;
				public static final VelocityMeasPeriod SPEED_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_1Ms;
				public static final int SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;

				// Position Constraints
				public static final double INIT_HEADING = 180.0;
				public static final double MIN_ROTATION_HEADING = 110.0;
				public static final double MAX_ROTATION_HEADING = 250.0;

				// Power Constraints
				public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
				public static final double SATURATION_VOLTAGE = 8.0;
				public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
				public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
				public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 48.0;
				public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
						* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
				public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

				// Error Tolerance
				public static final int CONTROL_ERROR_TOLERANCE = 0; // in encoder units
				public static final double HEADING_ON_TARGET_ERROR_TOLERANCE = 360.0 / 128.0;

				// PID Parameters
				public static final double PID0_KP = 0.875;
				public static final double PID0_KI = 0.0;
				public static final double PID0_KD = 4.0;
				public static final double PID0_KF = 1023.0 / MAX_SPEED;

				// Communication Parameters
				public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;

				// Mechanical Constants
				public static final int ENCODER_RESOLUTION = 2048;
				public static final double ENCODER_TO_TURRET_RATIO = 34.2857; // the number of rotations the encoder undergoes for every (yaw) rotation of the turret
				public static final double ENCODER_UNITS_PER_TURRET_REVOLUTION = ENCODER_RESOLUTION
						* ENCODER_TO_TURRET_RATIO;
				public static final double ENCODER_UNITS_PER_DEGREE = ENCODER_UNITS_PER_TURRET_REVOLUTION / 360.0;
			}

			public static class PitchGimbal
			{
				// Control Inversion
				public static final boolean INVERT_MOTOR = true;
				public static final boolean INVERT_SENSOR = false;

				// Speed Constants
				public static final double SPEED_RESERVE_FACTOR = 1.0 / 16.0;
				public static final double MAX_SPEED = 5100.0; // in encoder units / 0.1s
				public static final double MAX_CRUISE_SPEED = MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
				public static final double MAX_ACCELERATION = MAX_SPEED * 4.0; // estimated, in encoder units / 0.1s^2
				public static final VelocityMeasPeriod SPEED_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_1Ms;
				public static final int SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;

				// Power Constraints
				public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
				public static final double SATURATION_VOLTAGE = 8.0;
				public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
				public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
				public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 24.0;
				public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
						* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
				public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

				// Error Tolerance
				public static final int CONTROL_ERROR_TOLERANCE = 0; // in encoder units
				public static final double HEADING_ON_TARGET_ERROR_TOLERANCE = 360.0 / 128.0;

				// PID Parameters
				public static final double PID0_KP = 0.5;
				public static final double PID0_KI = 0.0;
				public static final double PID0_KD = 12.0;
				public static final double PID0_KF = 1023.0 / MAX_SPEED;

				// Communication Parameters
				public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;

				// Mechanical Constants
				public static final int ENCODER_RESOLUTION = 4096;
				public static final double ENCODER_POSITION_RESERVE_RATIO_LOW = 1.0 / 8.0;
				public static final double ENCODER_POSITION_RESERVE_RATIO_HIGH = 0.0;
				public static final int ENCODER_POSITION_LOW = 0;
				public static final int ENCODER_POSITION_HIGH = 3052 + 117;//4690;
				public static final double LAUNCH_ANGLE_MIN = 90.0 - 65; //65;
				public static final double LAUNCH_ANGLE_MAX = 90.0 - 39; //+ /*correction*/ 11.25;
				public static final double LAUNCH_ANGLE_OFFSET = LAUNCH_ANGLE_MAX;
				public static final double ENCODER_UNITS_PER_DEGREE = (ENCODER_POSITION_HIGH - ENCODER_POSITION_LOW)
						/ (LAUNCH_ANGLE_MIN - LAUNCH_ANGLE_MAX);
				public static final int ENCODER_POSITION_OPERATIONAL_LOW = Util
						.roundToInt((ENCODER_POSITION_HIGH - ENCODER_POSITION_LOW) * ENCODER_POSITION_RESERVE_RATIO_LOW
								+ ENCODER_POSITION_LOW);
				public static final int ENCODER_POSITION_OPERATIONAL_HIGH = Util.roundToInt(
						(ENCODER_POSITION_HIGH - ENCODER_POSITION_LOW) * (1.0 - ENCODER_POSITION_RESERVE_RATIO_HIGH)
								+ ENCODER_POSITION_LOW);
				public static final int ENCODER_POSITION_SOFT_LIMIT_LOW = Util
						.roundToInt((ENCODER_POSITION_LOW + ENCODER_POSITION_OPERATIONAL_LOW) / 2.0);
				public static final int ENCODER_POSITION_SOFT_LIMIT_HIGH = Util
						.roundToInt((ENCODER_POSITION_HIGH + ENCODER_POSITION_OPERATIONAL_HIGH) / 2.0);
			}
		}

		public static class BallDrive
		{
			// Control Inversion
			public static final boolean INVERT_LEFT_MOTOR = true;
			public static final boolean INVERT_RIGHT_MOTOR = false;

			// Speed Constants
			public static final double REVERSE_OPEN_LOOP_OUTPUT = 0.0625;
			public static final double SPEED_RESERVE_FACTOR = 1.0 / 16.0;
			public static final double MAX_SPEED = 17200.0; // in encoder units / 0.1s
			public static final double MAX_CRUISE_SPEED = MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
			public static final VelocityMeasPeriod SPEED_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_1Ms;
			public static final int SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;

			// Power Constraints
			public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
			public static final double SATURATION_VOLTAGE = 10.0;
			public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
			public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
			public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 192.0;
			public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
					* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
			public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

			// Error Tolerance
			public static final int CONTROL_ERROR_TOLERANCE = 0; // in encoder units
			public static final double SURFACE_SPEED_ON_TARGET_ERROR_TOLERANCE = 16.0;

			// PID Parameters
			public static final double PID0_KP = 0.5;
			public static final double PID0_KI = 0.0;
			public static final double PID0_KD = 0.0;
			public static final double PID0_KF = 1023.0 / MAX_SPEED;

			// Communication Parameters
			public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;

			// Mechanical Constants
			public static final double WHEEL_DIAMETER = 4.0;
			public static final int ENCODER_RESOLUTION = 2048;
			public static final double ENCODER_TO_FLYWHEEL_RATIO = 1 / 1.5; // the number of rotations the encoder undergoes for every rotation of the flywheel
			public static final double ENCODER_UNITS_PER_FLYWHEEL_REVOLUTION = ENCODER_RESOLUTION
					* ENCODER_TO_FLYWHEEL_RATIO;
			public static final double ENCODER_UNITS_PER_INCH = ENCODER_UNITS_PER_FLYWHEEL_REVOLUTION
					/ (Math.PI * WHEEL_DIAMETER);
			public static final double PROJECTILE_SPEED_FACTOR = 0.921875; // the initial speed of the projectile in terms of the surface speed of the flywheel
		}

		public static class BallFeeder
		{
			// Control Inversion
			public static final boolean INVERT_MOTOR = false;

			// Speed Constants
			public static final double INJECT_OPEN_LOOP_OUTPUT = 0.75;
			public static final double EJECT_OPEN_LOOP_OUTPUT = -0.5;

			// Power Constraints
			public static final boolean ENABLE_VOLTAGE_COMPENSATION = false;
			public static final double SATURATION_VOLTAGE = 12.0;
			public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
			public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
			public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 64.0;
			public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
					* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
			public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

			// Communication Parameters
			public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;
		}

		public static class Spindex
		{
			// Control Inversion
			public static final boolean INVERT_MOTOR = true;

			// Speed Constants
			public static final double SLOW_SPIN_OPEN_LOOP_OUTPUT = 0.125;
			public static final double FAST_SPIN_OPEN_LOOP_OUTPUT = 0.225;
			public static final double ANTI_JAM_OPEN_LOOP_OUTPUT = 0.3;

			// Power Constraints
			public static final boolean ENABLE_VOLTAGE_COMPENSATION = false;
			public static final double SATURATION_VOLTAGE = 12.0;
			public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
			public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
			public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 25.0;
			public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
					* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
			public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

			// Auto-Reverse Parameters
			public static final double AUTO_REVERSE_THRESHOLD_CURRENT_SLOW = 10.0;
			public static final double AUTO_REVERSE_THRESHOLD_CURRENT_FAST = 18.0;
			public static final double AUTO_REVERSE_MIN_PERIOD = 1.0;

			// Communication Parameters
			public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;
		}
	}

	public static class Collector
	{
		// Initial States
		public static final boolean INITIAL_ELEVATION = true;

		// Control Inversion
		public static final boolean INVERT_MOTOR = false;

		// Speed Constants
		public static final double INJECT_OPEN_LOOP_OUTPUT = 0.5;
		public static final double EJECT_OPEN_LOOP_OUTPUT = 0.5;

		// Power Constraints
		public static final boolean ENABLE_VOLTAGE_COMPENSATION = false;
		public static final double SATURATION_VOLTAGE = 12.0;
		public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
		public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
		public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 24.0;
		public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
				* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
		public static final double CURRENT_LIMITER_TRIGGER_DELAY = 0.0;

		// Communication Parameters
		public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;
	}

	public static class Climber
	{
		// Control Inversion
		public static final boolean INVERT_MOTOR = true;

		// Speed Constants
		public static final double CLIMBER_EXTEND_OPEN_LOOP_OUTPUT = 0.25;
		public static final double CLIMBER_RETRACT_OPEN_LOOP_OUTPUT = 0.75;

		// Power Constraints
		public static final boolean ENABLE_VOLTAGE_COMPENSATION = false;
		public static final double SATURATION_VOLTAGE = 12.0;
		public static final int VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE = 1;
		public static final double CURRENT_LIMITER_RESERVE_RATIO = 0.0;
		public static final double CURRENT_LIMITER_THRESHOLD_CURRENT = 48.0;
		public static final double CURRENT_LIMITER_TARGET_CURRENT = CURRENT_LIMITER_THRESHOLD_CURRENT
				* (1.0 - CURRENT_LIMITER_RESERVE_RATIO);
		public static final double CURRENT_LIMITER_TRIGGER_DELAY = 4.0;

		// Communication Parameters
		public static final int STATUS_FRAME_PERIOD = Looper.DELTA_TIME_MS;
	}

	private Constants()
	{
	}
}
