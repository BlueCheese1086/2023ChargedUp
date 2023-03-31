package frc.robot.Util;

public class Kalman1D {
    private double state;
    private double covariance;
    private double processNoise;
    private double measurementNoise;

    public Kalman1D(double initial_state, double initial_covariance, double process_noise, double measurement_noise) {
        this.state = initial_state;
        this.covariance = initial_covariance;
        this.processNoise = process_noise;
        this.measurementNoise = measurement_noise;
    }

    public double predict() {
        this.covariance = this.covariance + this.processNoise;
        return this.state;
    }

    public void update(double measurement) {
        double kalmanGain = this.covariance / (this.covariance + this.measurementNoise);
        this.state = this.state + kalmanGain * (measurement - this.state);
        this.covariance = (1 - kalmanGain) * this.covariance;
    }

    public void example() {
        double initial_state = 0;
        double initial_covariance = 1;
        double process_noise = 0.1;
        double measurement_noise = 0.5;
        Kalman1D kf = new Kalman1D(initial_state, initial_covariance, process_noise, measurement_noise);

        double[] measurements = {1, 0.9, 1.1, 0.95, 1.05};  // Example measurements
        double[] filteredStates = new double[measurements.length];

        for (int i = 0; i < measurements.length; i++) {
            kf.predict();
            kf.update(measurements[i]);
            filteredStates[i] = kf.state;
        }

        System.out.print("Filtered states: ");
        for (double filteredState : filteredStates) {
            System.out.print(filteredState + " ");
        }
    }
}
