package frc.robot.util;

import java.util.HashMap;
import java.util.logging.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class LoggerUtil {
    public static final boolean globalDebugMode = true;
    boolean debugMode;
    String name;
    NetworkTable table;
    HashMap<String, BooleanPublisher> boolTopics = new HashMap<>();
    HashMap<String, IntegerPublisher> intTopics = new HashMap<>();
    HashMap<String, DoublePublisher> doubleTopics = new HashMap<>();
    HashMap<String, StructPublisher<Pose2d>> pose2dTopics = new HashMap<>();

    public LoggerUtil(String name) {
        this(name, false);
    }
    public LoggerUtil(String name, boolean subsystemDebug) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        debugMode = globalDebugMode || subsystemDebug;
    }

    public void put(String name, boolean value) {
        put(name, value, false);
    }
    public void put(String topic, boolean value, boolean critical) {
        if (debugMode || critical) {
            if (!boolTopics.containsKey(topic)) {
                boolTopics.put(topic, table.getBooleanTopic(topic).publish());
            }
            boolTopics.get(topic).set(value);
        }
        SignalLogger.writeBoolean(name + "/" + topic, value);
    }

    public void put(String name, int value) {
        put(name, value, false);
    }
    public void put(String topic, int value, boolean critical) {
        if (debugMode || critical) {
            if (!intTopics.containsKey(topic)) {
                intTopics.put(topic, table.getIntegerTopic(topic).publish());
            }
            intTopics.get(topic).set(value);
        }
        SignalLogger.writeInteger(name + "/" + topic, value);
    }

    public void put(String name, double value) {
        put(name, value, false);
    }
    public void put(String topic, double value, boolean critical) {
        if (debugMode || critical) {
            if (!doubleTopics.containsKey(topic)) {
                doubleTopics.put(topic, table.getDoubleTopic(topic).publish());
            }
            doubleTopics.get(topic).set(value);
        }
        SignalLogger.writeDouble(name + "/" + topic, value);
    }

    public void put(String name, Pose2d value) {
        put(name, value, false);
    }
    public void put(String topic, Pose2d value, boolean critical) {
        if (debugMode || critical) {
            if (!pose2dTopics.containsKey(topic)) {
                pose2dTopics.put(topic, table.getStructTopic(topic, Pose2d.struct).publish());
            }
            pose2dTopics.get(topic).set(value);
        }
        SignalLogger.writeStruct(name + "/" + topic, Pose2d.struct, value);
    }
}
