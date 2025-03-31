#include <iostream>
#include <random>
#include <cmath>
#include <algorithm> // For std::max

// Function to sample a triangular distribution
double sample_triangular_distribution(double b) {
    // Generate two random numbers in the range [-b, b]
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-b, b);

    double rand1 = dis(gen);
    double rand2 = dis(gen);

    // Triangular distribution formula
    return (std::sqrt(6) / 2.0) * (rand1 + rand2);
}

// Function to calculate errors in Vx and theta
void calculate_errors(double V_measured, double theta_measured, 
                      double alpha1, double alpha2, double alpha3, double alpha4) {
    // Evaluate variances
    double b_Vx = alpha1 * V_measured * V_measured + alpha2 * theta_measured * theta_measured;
    double b_theta = alpha3 * V_measured * V_measured + alpha4 * theta_measured * theta_measured;

    // Sample triangular distribution for the evaluated variances
    double error_Vx = sample_triangular_distribution(b_Vx);
    double error_theta = sample_triangular_distribution(b_theta);

    // Print results
    std::cout << "Error in Vx (under distribution): " << error_Vx << std::endl;
    std::cout << "Error in theta (under distribution): " << error_theta << std::endl;
}

int main() {
    // Measured values
    double V_measured = 0.01;  // Example linear velocity (m/s)
    double theta_measured = 0.5; // Example angular velocity (rad/s)
    
    // Alpha coefficients
    double alpha1 = 0.01;
    double alpha2 = 0.025;
    double alpha3 = 0.001;
    double alpha4 = 0.1;

    // Calculate and print errors
    calculate_errors(V_measured, theta_measured, alpha1, alpha2, alpha3, alpha4);

    return 0;
}
