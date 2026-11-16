/**
 * @file Madgwick.h
 * @brief Madgwick AHRS (Attitude and Heading Reference System) algorithm implementation
 * @author Sebastian O.H. Madgwick (original), EdgeFlyte LLC (integration)
 * @date 2025 (integration), 2011 (original algorithm)
 * @copyright Original algorithm: GNU General Public License
 * 
 * Implementation of Madgwick's IMU and AHRS algorithms for sensor fusion.
 * This provides orientation estimation by fusing accelerometer, gyroscope, 
 * and magnetometer data using a complementary filter approach.
 * 
 * Key features:
 * - Quaternion-based orientation representation
 * - Computationally efficient (suitable for microcontrollers)
 * - Automatic gyroscope bias compensation
 * - Configurable filter gain (beta parameter)
 * - Support for both 6-DOF (IMU) and 9-DOF (AHRS) operation
 * 
 * Original algorithm reference:
 * S.O.H. Madgwick, A.J.L. Harrison, and R. Vaidyanathan,
 * "Estimation of IMU and MARG orientation using a gradient descent algorithm,"
 * IEEE International Conference on Rehabilitation Robotics, 2011.
 * 
 * See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * 
 * Version History:
 * - 29/09/2011: SOH Madgwick - Initial release
 * - 02/10/2011: SOH Madgwick - Optimised for reduced CPU load
 * - 2025: EdgeFlyte LLC - Integration and documentation updates
 */
#ifndef __Madgwick_h__
#define __Madgwick_h__
#include <math.h>
#include <stdint.h>

/**
 * @brief Madgwick AHRS filter class for sensor fusion and orientation estimation
 * 
 * This class implements the Madgwick algorithm for fusing IMU sensor data
 * (accelerometer, gyroscope, magnetometer) to estimate device orientation
 * represented as a quaternion.
 * 
 * The filter uses a gradient descent approach to minimize the error between
 * predicted and measured directions of gravity and magnetic field.
 */
class Madgwick {
  private:
    float delta_t = 0;           ///< Time step for integration (calculated automatically)
    uint32_t now = 0;            ///< Current timestamp in microseconds
    uint32_t last_update = 0;    ///< Previous update timestamp for delta_t calculation
    
    /**
     * @brief Fast inverse square root approximation (Quake algorithm)
     * @param x Input value
     * @return Approximation of 1/sqrt(x)
     * @note Uses bit manipulation for fast computation on embedded systems
     */
    static float invSqrt(float x);
    
    float beta;                  ///< Algorithm gain parameter (controls convergence rate)
    float q0;                    ///< Quaternion scalar component (w)
    float q1;                    ///< Quaternion vector component (x)
    float q2;                    ///< Quaternion vector component (y)
    float q3;                    ///< Quaternion vector component (z)

  public:
    /**
     * @brief Default constructor - initializes quaternion to identity
     */
    Madgwick(void);
    
    /**
     * @brief Change the filter gain parameter during runtime
     * @param newBeta New beta value (typically 0.1 to 2.0)
     * @note Higher beta = faster convergence but more noise sensitivity
     */
    void changeBeta(float newBeta) { beta = newBeta; }
    
    /**
     * @brief Initialize the filter with a specific beta value
     * @param confBeta Beta parameter for filter tuning
     * @note Recommended values: 0.1 (smooth) to 2.0 (responsive)
     */
    void begin(float confBeta) { beta = confBeta; }
    
    /**
     * @brief Update filter with 9-DOF sensor data (full AHRS)
     * @param gx Gyroscope X-axis (degrees/second)
     * @param gy Gyroscope Y-axis (degrees/second)
     * @param gz Gyroscope Z-axis (degrees/second)
     * @param ax Accelerometer X-axis (any units, will be normalized)
     * @param ay Accelerometer Y-axis (any units, will be normalized)
     * @param az Accelerometer Z-axis (any units, will be normalized)
     * @param mx Magnetometer X-axis (any units, will be normalized)
     * @param my Magnetometer Y-axis (any units, will be normalized)
     * @param mz Magnetometer Z-axis (any units, will be normalized)
     * @note Call this regularly (e.g., 100Hz) for best performance
     */
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    
    /**
     * @brief Update filter with 6-DOF sensor data (IMU only, no magnetometer)
     * @param gx Gyroscope X-axis (degrees/second)
     * @param gy Gyroscope Y-axis (degrees/second)
     * @param gz Gyroscope Z-axis (degrees/second)
     * @param ax Accelerometer X-axis (any units, will be normalized)
     * @param ay Accelerometer Y-axis (any units, will be normalized)
     * @param az Accelerometer Z-axis (any units, will be normalized)
     * @note Use when magnetometer is unavailable or unreliable
     */
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    /**
     * @brief Get quaternion W component (scalar part)
     * @return Quaternion W component
     */
    float getQuatW() {
      return q0;
    }

    /**
     * @brief Get quaternion X component (vector part)
     * @return Quaternion X component
     */
    float getQuatX() {
      return q1;
    }

    /**
     * @brief Get quaternion Y component (vector part)
     * @return Quaternion Y component
     */
    float getQuatY() {
      return q2;
    }

    /**
     * @brief Get quaternion Z component (vector part)
     * @return Quaternion Z component
     */
    float getQuatZ() {
      return q3;
    }

};
#endif
