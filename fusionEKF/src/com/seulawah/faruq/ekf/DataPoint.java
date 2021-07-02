package com.seulawah.faruq.ekf;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

enum DataPointType {RADAR, LIDAR,STATE}

public class DataPoint {
    private long timestamp;
    private boolean initialized;
    private DataPointType data_type;
    private RealVector raw;

    public DataPoint() {
        this.initialized = false;
    }

    public DataPoint(final long timestamp, final DataPointType data_type, final RealVector raw) {
        this.set(timestamp, data_type, raw);
    }

    public void set(final long timestamp, final DataPointType data_type, final RealVector raw) {
        this.timestamp = timestamp;
        this.data_type = data_type;
        this.raw = raw;
        this.initialized = true;
    }

    public final RealVector get() {
        return this.raw;
    }

    public final RealVector get_state(){
        RealVector state = new ArrayRealVector(4);
        Tools tools = new Tools();

        if (this.data_type == DataPointType.RADAR) {
            state = tools.convert_polar_to_cartesian(this.raw);
        }
        else if (this.data_type == DataPointType.LIDAR) {
            double x = this.raw.getEntry(0);
            double y = this.raw.getEntry(1);
            state.setEntry(0, x);
            state.setEntry(1, y);
        }
        else if (this.data_type == DataPointType.STATE) {
            state = this.raw;
        }
        return state;
    }

    public final long get_timestamp() {
        return this.timestamp;
    }

    public final DataPointType get_type() {
        return this.data_type;
    }

    public final void print() {
        if (this.initialized) {
            System.out.println("Timestamp: " + this.timestamp);
            System.out.println("Sensor ID: " + this.data_type.ordinal() + " (RADAR = 0 | STATE = 1)");
            System.out.println("Raw data: ");
            System.out.println(this.raw);

        } else {
            System.out.println("DataPoint is not initialized");
        }
    }
}