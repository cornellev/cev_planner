#pragma once

#include <cmath>
#include <random>

namespace cev_planner {
    /**
     * @brief Restrict an angle to the range [-pi, pi]
     *
     * @param angle Angle in radians
     * @return float Restricted angle
     */
    static float restrict_angle(float angle) {
        float new_angle = std::fmod(angle, 2 * M_PI);

        if (new_angle >= M_PI) {
            new_angle -= 2 * M_PI;
        } else if (new_angle < -M_PI) {
            new_angle += 2 * M_PI;
        }

        return new_angle;
    }
    
    class Random {
        public:
        explicit Random(double min = 0.0, double max = 1.0)
            : gen(rd()), default_distrib(min, max), default_distrib_int((int)(round(min)), (int)(round(max))) {}

        /**
         * @brief Generate a random integer in the range [min, max]
         *
         * @param min Minimum value (inclusive)
         * @param max Maximum value (inclusive)
         * @return int Random integer
         */
        int Int(int min, int max) {
            std::uniform_int_distribution<int> distrib(min, max);
            return distrib(gen);
        }

        /**
         * @brief Generate a random double in the range [min, max]
         *
         * @param min Minimum value (inclusive)
         * @param max Maximum value (inclusive)
         * @return double Random double
         */
        double Double(double min, double max) {
            std::uniform_real_distribution<double> distrib(min, max);
            return distrib(gen);
        }

        /**
         * @brief Generate a random double from the default distribution (0 to 1 by default)
         *
         * @return double Random double
         */
        double Uniform() {
            return default_distrib(gen);
        }
    
        int Int() {
            return default_distrib_int(gen);
        }

        private:
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_real_distribution<double> default_distrib;
        std::uniform_int_distribution<int> default_distrib_int;
        };
}  // namespace cev_planner