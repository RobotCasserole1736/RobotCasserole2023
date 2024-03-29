package frc.lib.Signal;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;



public class Signal {

    String name;
    String units;
    DoubleTopic nt4ValTopic;
    DoublePublisher nt4ValPublisher;
    DoubleLogEntry logEntry;

    /**
     * Class which describes one line on a plot
     * 
     * @param name_in  String of what to call the signal (human readable)
     * @param units_in units the signal is in.
     */
    public Signal(String name_in, String units_in) {
        name = name_in;
        units = units_in;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt4ValTopic = inst.getDoubleTopic(this.getNT4ValueTopicName());
        logEntry = new DoubleLogEntry(DataLogManager.getLog(), name_in);


        //The goal of a signal is to record the value of a variable every loop, for debugging down to loop-to-loop changes
        // Therefor we do want to send all vlaues over the network, and we do want to keep any duplicates.
        nt4ValPublisher = nt4ValTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        nt4ValTopic.setProperties("{ \"units\" : \"" + units + "\"}");


        SignalWrangler.getInstance().register(this);
    }

    /**
     * Adds a new sample to the signal queue. It is intended that the controls code
     * would call this once per loop to add a new datapoint to the real-time graph.
     * 
     * The boolean version converts true to 1.0 and false to 0.0.
     * 
     * @param time_in
     * @param value_in
     */
    public void addSample(double time_in_sec, boolean value_in) {
        double value = value_in? 1.0: 0.0;
        long timestamp_us = Math.round(time_in_sec*1000000l);
        nt4ValPublisher.set(value, timestamp_us);
        logEntry.append(value, timestamp_us);
    }

    /**
     * Adds a new sample to the signal queue. It is intended that the controls code
     * would call this once per loop to add a new datapoint to the real-time graph.
     * 
     * @param time_in
     * @param value_in
     */
    public void addSample(double time_in_sec, double value_in) {
        long timestamp_us = Math.round(time_in_sec*1000000l);
        nt4ValPublisher.set(value_in, timestamp_us);
        logEntry.append(value_in, timestamp_us);
    }

    /**
     * @return The User-friendly name of the signal
     */
    public String getName() {
        return name;
    }

    /**
     * @return The name of the units the signal is measured in.
     */
    public String getUnits() {
        return units;
    }

    public String getNT4ValueTopicName(){ return SignalUtils.nameToNT4ValueTopic(this.name); }

}
