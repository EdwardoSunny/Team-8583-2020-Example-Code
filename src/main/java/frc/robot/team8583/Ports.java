package frc.robot.team8583;

public class Ports
{
    public static class Can
    {
        // Swerve Drive
        public static final int FRONT_LEFT_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_ROTATION_MOTOR = 1;
        public static final int FRONT_LEFT_ROTATION_SENSOR = 8;

        public static final int REAR_LEFT_DRIVE_MOTOR = 2;
        public static final int REAR_LEFT_ROTATION_MOTOR = 3;
        public static final int REAR_LEFT_ROTATION_SENSOR = 9;

        public static final int REAR_RIGHT_DRIVE_MOTOR = 4;
        public static final int REAR_RIGHT_ROTATION_MOTOR = 5;
        public static final int REAR_RIGHT_ROTATION_SENSOR = 10;

        public static final int FRONT_RIGHT_DRIVE_MOTOR = 6;
        public static final int FRONT_RIGHT_ROTATION_MOTOR = 7;
        public static final int FRONT_RIGHT_ROTATION_SENSOR = 11;

        // Shooter
        public static final int TURRET_PITCH_MOTOR = 12;
        public static final int TURRET_YAW_MOTOR = 13;

        public static final int BALL_DRIVE_LEFT_MOTOR = 14;
        public static final int BALL_DRIVE_RIGHT_MOTOR = 15;

        public static final int SPINDEX_MOTOR = 8;
        public static final int BALL_FEEDER_MOTOR = 9;

        // Collector
        public static final int COLLECTOR_MOTOR = 11;

        // Climber
        public static final int CLIMBER_MOTOR = 10;

        // Pneumatics
        public static final int PCM = 0;

        // Sensors
        public static final int PIGEON = 0;

    }

    public static class Pcm
    {
        // Collector
        public static final int COLLECTOR_ELEVATE_PULL = 2;
        public static final int COLLECTOR_ELEVATE_PUSH = 3;

        // Climber
        public static final int CLIMBER_LOCK_PULL = 0;
        public static final int CLIMBER_LOCK_PUSH = 1;
    }
}
