/*
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \defgroup Filters Filters
 *
 * @ingroup ctrlLibRT
 *
 * Classes for filtering, modified to avoid non-realtime behaviour.
 *
 * \author Ugo Pattacini, Silvio Traversaro.
 *
 */

#ifndef RT_FILTERS_H
#define RT_FILTERS_H

#include <Eigen/Dense>
#include <memory>

namespace iCub
{

namespace ctrl
{

namespace realTime
{

/**
* \ingroup Filters
*
* IIR and FIR.
*/
class Filter
{
protected:
   Eigen::VectorXd b;
   Eigen::VectorXd a;
   Eigen::VectorXd y;

   Eigen::MatrixXd uold; ///< Matrix of past inputs: each column is a past sample
   int uold_last_column_sample;

   Eigen::MatrixXd yold; ///< Matrix of past outpts: each columns is a past output
   int yold_last_column_sample;

   size_t n;
   size_t m;

public:
   /**
   * Creates a filter with specified numerator and denominator
   * coefficients.
   * @param num vector of numerator elements given as increasing
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing
   *            power of z^-1.
   * @param y0 initial output.
   * @note den[0] shall not be 0.
   */
    Filter(const Eigen::Ref<const Eigen::VectorXd>& num,
           const Eigen::Ref<const Eigen::VectorXd>& den,
           const Eigen::Ref<const Eigen::VectorXd>& y0);

    /**
     * Internal state reset.
     * @param y0 new internal state.
     */
    void init(const Eigen::Ref<const Eigen::VectorXd>& y0);

    /**
     * Internal state reset for filter with zero gain.
     * @param y0 new internal state.
     * @param u0 expected next input.
     * @note The gain of a digital filter is the sum of the coefficients of its
     *       numerator divided by the sum of the coefficients of its denumerator.
     */
    void
    init(const Eigen::Ref<const Eigen::VectorXd>& y0, const Eigen::Ref<const Eigen::VectorXd>& u0);

    /**
     * Returns the current filter coefficients.
     * @param num vector of numerator elements returned as increasing
     *            power of z^-1.
     * @param den vector of denominator elements returned as
     *            increasing power of z^-1.
     */
    void getCoeffs(Eigen::Ref<Eigen::VectorXd> num, Eigen::Ref<Eigen::VectorXd> den);

    /**
     * Sets new filter coefficients.
     * @param num vector of numerator elements given as increasing
     *            power of z^-1.
     * @param den vector of denominator elements given as increasing
     *            power of z^-1.
     * @note den[0] shall not be 0.
     * @note the internal state is reinitialized to the current
     *       output.
     */
    void setCoeffs(const Eigen::Ref<const Eigen::VectorXd>& num,
                   const Eigen::Ref<const Eigen::VectorXd>& den);

    /**
     * Modifies the values of existing filter coefficients without
     * varying their lengths.
     * @param num vector of numerator elements given as increasing
     *            power of z^-1.
     * @param den vector of denominator elements given as increasing
     *            power of z^-1.
     * @return true/false on success/fail.
     * @note den[0] shall not be 0.
     * @note the adjustment is carried out iff num.size() and
     *       den.size() match the existing numerator and denominator
     *       lengths.
     */
    bool adjustCoeffs(const Eigen::Ref<const Eigen::VectorXd>& num,
                      const Eigen::Ref<const Eigen::VectorXd>& den);

    /**
     * Performs filtering on the actual input.
     * @param u reference to the actual input.
     * @return a reference the corresponding output.
     * @note the returned reference is valid till any new call to filt.
     */
    Eigen::Ref<const Eigen::VectorXd> filt(const Eigen::Ref<const Eigen::VectorXd>& u);

    /**
     * Return the reference to the current filter output.
     * @return reference to the filter output.
     */
    Eigen::Ref<const Eigen::VectorXd> output() const
    {
        return y;
    }
};

/**
* \ingroup Filters
*
* First order low pass filter implementing the transfer function
* H(s) = \frac{1}{1+\tau s}
*
*/
class FirstOrderLowPassFilter
{
protected:
    std::unique_ptr<Filter> filter;         // low pass filter
    double fc;              // cut frequency
    double Ts;              // sample time
    Eigen::VectorXd y;    // filter current output

    void computeCoeff();

public:
    /**
    * Creates a filter with specified parameters
    * @param cutFrequency cut frequency (Hz).
    * @param sampleTime sample time (s).
    * @param y0 initial output.
    */
    FirstOrderLowPassFilter(const double cutFrequency, const double sampleTime,
                            const Eigen::Ref<const Eigen::VectorXd> &y0=Eigen::VectorXd::Zero(1));
    /**
    * Internal state reset.
    * @param y0 new internal state.
    */
    void init(const Eigen::Ref<const Eigen::VectorXd> &y0);

    /**
    * Change the cut frequency of the filter.
    * @param cutFrequency the new cut frequency (Hz).
    */
    bool setCutFrequency(const double cutFrequency);

    /**
    * Change the sample time of the filter.
    * @param sampleTime the new sample time (s).
    */
    bool setSampleTime(const double sampleTime);

    /**
    * Retrieve the cut frequency of the filter.
    * @return the cut frequency (Hz).
    */
    double getCutFrequency() { return fc; }

    /**
    * Retrieve the sample time of the filter.
    * @return the sample time (s).
    */
    double getSampleTime() { return Ts; }

    /**
    * Performs filtering on the actual input.
    * @param u reference to the actual input.
    * @return the corresponding output.
    */
    Eigen::Ref<const Eigen::VectorXd> filt(const Eigen::Ref<const Eigen::VectorXd> &u);

    /**
    * Return current filter output.
    * @return the filter output.
    */
    Eigen::Ref<const Eigen::VectorXd> output() const
    {
        return y;
    }
};


}

}

}

#endif

