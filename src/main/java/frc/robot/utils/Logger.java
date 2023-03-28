package frc.robot.utils;

public class Logger {
    public enum Level {
        TRACE(0),
        DEBUG(1),
        INFO(2),
        WARNING(3),
        ERROR(4);

        public final int value;

        private Level(int value){
            this.value = value;
        }
    };

    private static Level m_logLevel = Level.TRACE;

    public static void setLogLevel(Level logLevel) {
        m_logLevel = logLevel;
    }

    /**
     * Log to stdout, prepending the class and method doing the logging
     * @param msg
     */
    public static void Log (Level level, String msg) {
        if (m_logLevel.value >= level.value) {
            System.out.println(level + " : " + 
                Thread.currentThread().getStackTrace()[2].getClassName() + "::" +
                Thread.currentThread().getStackTrace()[2].getMethodName() + " -- " +
                msg);
        }
    }

    public static void Log (String msg) {
        Log(Level.DEBUG, msg);
    }
}
