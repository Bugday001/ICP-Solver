#ifndef LIDAR_GTSAM_
#define LIDAR_GTSAM_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>
#include <cmath>
using namespace gtsam;

namespace XICP
{
    class IcpFactor : public NoiseModelFactor1<Pose3>
    {
    public:
        /**
         * @brief Construct a new Icp Factor object
         *
         * @param noiseModel
         * @param key
         * @param measurement
         * @param p
         */
        IcpFactor(const SharedNoiseModel &noiseModel, Key key,
                  const Point3 &measurement, const Point3 &p)
            : NoiseModelFactor1(noiseModel, key), measurement_(measurement), p_(p)
        {
        }

        /**
         * @brief Destroy the Icp Factor object
         *
         */
        virtual ~IcpFactor()
        {
        }

        /**
         * @brief
         *
         * @param x
         * @param H
         * @return Vector
         */
        virtual Vector evaluateError(
            const Pose3 &T, boost::optional<Matrix &> H = boost::none) const override
        {
            // construct prediction and compute jacobian
            gtsam::Matrix36 Hpose;
            const Point3 prediction = T.transformFrom(p_, Hpose);

            if (H)
            {
                *H = Hpose;
            }

            return prediction - measurement_;
        }

    private:
        Point3 measurement_;
        Point3 p_;
    };
} // namespace XICP

#endif