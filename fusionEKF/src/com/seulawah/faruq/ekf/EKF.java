package com.seulawah.faruq.ekf;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.MatrixUtils;

public class EKF {

    private final int n =4;
    private final double ax = 9.0d; //5
    private final double ay = 9.0d; //5
    private boolean initialized;
    private  long timestamp;
    private RealMatrix P;
    private RealMatrix F;
    private RealMatrix Q;
    private RealMatrix R_radar;
    private RealMatrix R_lidar;
    private RealMatrix H_lidar;
    //private RealMatrix R = new Array2DRowRealMatrix();
    private KalmanFilter KF = new KalmanFilter();

    public EKF() {
        this.initialized = false;
        this.R_radar = new Array2DRowRealMatrix();
        this.R_lidar = new Array2DRowRealMatrix();
        this.F = MatrixUtils.createRealIdentityMatrix(n);
        this.Q = MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(0);  //matrix zero Q

        this.R_radar = new Array2DRowRealMatrix(new double[][] {
                {0.09, 0.0, 0.0},
                {0, 0.0009, 0.0},
                {0, 0.0, 0.09}
        });

        this.R_lidar = new Array2DRowRealMatrix(new double[][] {
                {0.0225, 0.0},
                {0, 0.0225}
        });

        this.H_lidar = new Array2DRowRealMatrix(new double[][] {
                {1.0, 0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0, 0.0}
        });

        this.P = new Array2DRowRealMatrix(new double[][] {
                {1.0, 0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0, 0.0},
                {0.0, 0.0, 1000.0, 0.0},
                {0.0, 0.0, 0.0, 1000.0},
        });
    }

    public void updateQ(final double dt){
        final double dt2 = dt * dt;
        final double dt3 = dt * dt2;
        final double dt4 = dt * dt3;

        final double r11 = dt4 * this.ax / 4;
        final double r13 = dt3 * this.ax / 2;
        final double r22 = dt4 * this.ay / 4;
        final double r24 = dt3 * this.ay / 2;
        final double r31 = dt3 * this.ax / 2;
        final double r33 = dt2 * this.ax;
        final double r42 = dt3 * this.ay / 2;
        final double r44 = dt2 * this.ay;

        this.Q = new Array2DRowRealMatrix(new double[][] {
                {r11, 0.0, r13, 0.0},
                {0.0, r22, 0.0, r24},
                {r31, 0.0, r33, 0.0},
                {0.0, r42, 0.0, r44},
        });

        this.KF.setQ(Q);
    }

    public void start(final DataPoint data) {
        this.timestamp = data.get_timestamp();
        RealVector x = data.get_state();
        this.KF.start(this.n, x, this.P, this.F, this.Q);
        this.initialized = true;
    }

    public void compute(final DataPoint data) {
        final double dt  = (data.get_timestamp() - this.timestamp) / 1.e6;
        this.timestamp = data.get_timestamp();

        this.updateQ(dt);
        this.KF.updateF(dt);
        this.KF.predict();

        Tools tools = new Tools();
        final RealVector z = data.get();
        final RealVector x = this.KF.get();

        RealVector Hx;
        RealMatrix R;
        RealMatrix H;

        if (data.get_type() == DataPointType.RADAR) {
            RealVector s = data.get_state();
            H = tools.calculate_Jacobian(s);
            Hx = tools.convert_cartesian_to_polar(x);
            R = this.R_radar;

            this.KF.update(z, H, Hx, R);
        }
        else if (data.get_type() == DataPointType.LIDAR) {
            H = this.H_lidar;
            Hx = this.H_lidar.operate(x);
            R = this.R_lidar;

            this.KF.update(z, H, Hx, R);
        }
    }

    public void process (final DataPoint data) {
        if (this.initialized) {
            this.compute(data);
        } else {
            this.start(data);
        }
    }

    public final RealVector get() {
        return this.KF.get();
    }
}
