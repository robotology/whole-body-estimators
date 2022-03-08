/*
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * website: www.icub.org
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

#include "ctrlLibRT/filters.h"

#include <memory>

using namespace std;
using namespace iCub::ctrl::realTime;

/***************************************************************************/
Filter::Filter(const Eigen::Ref<const Eigen::VectorXd>& num,
               const Eigen::Ref<const Eigen::VectorXd>& den,
               const Eigen::Ref<const Eigen::VectorXd>& y0)
{
    b = num;
    a = den;

    m=b.size(); uold.resize(y0.size(),m-1);
    n=a.size(); yold.resize(y0.size(),m-1);

    init(y0);
}


/***************************************************************************/
void Filter::init(const Eigen::Ref<const Eigen::VectorXd>& y0)
{
    // otherwise use zero
    init(y0, Eigen::VectorXd::Zero(y0.size()));
}


/***************************************************************************/
void Filter::init(const Eigen::Ref<const Eigen::VectorXd>& y0,
                  const Eigen::Ref<const Eigen::VectorXd>& u0)
{
    // set all the element of u_init equal to zero
    Eigen::VectorXd u_init = Eigen::VectorXd::Zero(y0.size());
    Eigen::VectorXd y_init = y0;
    y=y0;

    const double sum_b = b.sum();
    const double sum_a = a.sum();

    if (abs(sum_b) > 1e-9) // if filter DC gain is not zero
        u_init = (sum_a / sum_b) * y0;
    else
    {
        // if filter gain is zero then you need to know in advance what
        // the next input is going to be for initializing (that is u0)
        // Note that, unless y0=0, the filter output is not going to be stable
        u_init = u0;
        if (abs(sum_a - a[0]) > 1e-9)
            y_init = a[0] / (a[0] - sum_a) * y;
        // if sum_a==a[0] then the filter can only be initialized to zero
    }

    for (int i = 0; i < yold.cols(); i++)
        yold.col(i) = y_init;
    yold_last_column_sample = 0;

    for (int i = 0; i < uold.cols(); i++)
        uold.col(i) = u_init;
    uold_last_column_sample = 0;
}

/***************************************************************************/
void Filter::getCoeffs(Eigen::Ref<Eigen::VectorXd> num, Eigen::Ref<Eigen::VectorXd> den)
{
    num = b;
    den = a;
}

/***************************************************************************/
void Filter::setCoeffs(const Eigen::Ref<const Eigen::VectorXd>& num,
                       const Eigen::Ref<const Eigen::VectorXd>& den)
{
    b = num;
    a = den;

    uold.setZero();
    yold.setZero();

    m=b.size(); uold.resize(y.size(),m-1);
    n=a.size(); yold.resize(y.size(),n-1);

    init(y);
}


/***************************************************************************/
bool Filter::adjustCoeffs(const Eigen::Ref<const Eigen::VectorXd>& num,
                          const Eigen::Ref<const Eigen::VectorXd>& den)
{
    if ((num.size()==(size_t)b.size()) && (den.size()==(size_t)a.size()))
    {
        b = num;
        a = den;

        return true;
    } else
        return false;
}


/***************************************************************************/
Eigen::Ref<const Eigen::VectorXd> Filter::filt(const Eigen::Ref<const Eigen::VectorXd>& u)
{
    y = b[0] * u;

    for (size_t i=1; i<m; i++)
    {
        y.noalias() += b[i] * uold.col((m - i + uold_last_column_sample) % (m - 1));
    }

    for (size_t i=1; i<n; i++)
    {
        y.noalias() -= a[i] * yold.col((n - i + yold_last_column_sample) % (n - 1));
    }

    y = (1.0 / a[0]) * y;

    uold_last_column_sample++;
    uold_last_column_sample = uold_last_column_sample % uold.cols();
    uold.col(uold_last_column_sample) = u;

    yold_last_column_sample++;
    yold_last_column_sample = yold_last_column_sample % yold.cols();
    yold.col(yold_last_column_sample) = y;

    return y;
}


/**********************************************************************/
FirstOrderLowPassFilter::FirstOrderLowPassFilter(const double cutFrequency,
                                                 const double sampleTime,
                                                 const Eigen::Ref<const Eigen::VectorXd> &y0)
{
    fc=cutFrequency;
    Ts=sampleTime;
    y=y0;
    computeCoeff();
}

/***************************************************************************/
void FirstOrderLowPassFilter::init(const Eigen::Ref<const Eigen::VectorXd> &y0)
{
    if (filter != nullptr)
        filter->init(y0);
}

/**********************************************************************/
bool FirstOrderLowPassFilter::setCutFrequency(const double cutFrequency)
{
    if (cutFrequency <= 0.0)
        return false;

    // Optimization : if the  cutoff frequency is exactly the same
    // do not update the coeffiencts
    // Note: in general the equality between two doubles is not a
    // reliable check, but in this case it make sense
    if (fc != cutFrequency)
    {
        fc = cutFrequency;
        computeCoeff();
    }

    return true;
}

/**********************************************************************/
bool FirstOrderLowPassFilter::setSampleTime(const double sampleTime)
{
    if (sampleTime <= 0.0)
        return false;

    Ts = sampleTime;
    computeCoeff();

    return true;
}

/**********************************************************************/
Eigen::Ref<const Eigen::VectorXd>
FirstOrderLowPassFilter::filt(const Eigen::Ref<const Eigen::VectorXd>& u)
{
    if (filter != nullptr)
        y = filter->filt(u);

    return y;
}

/**********************************************************************/
void FirstOrderLowPassFilter::computeCoeff()
{
    const double tau = 1.0 / (2.0 * M_PI * fc);
    const Eigen::Vector2d num = Eigen::Vector2d::Constant(Ts);
    Eigen::Vector2d den;
    den << 2.0 * tau + Ts, Ts - 2.0 * tau;

    if (filter!=nullptr)
        filter->adjustCoeffs(num,den);
    else
        filter = std::make_unique<Filter>(num, den, y);
}

