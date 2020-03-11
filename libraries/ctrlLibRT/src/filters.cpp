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

#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl::realTime;

/***************************************************************************/
Eigen::Map<Eigen::VectorXd> toEigen(yarp::sig::Vector & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}

Eigen::Map<const Eigen::VectorXd> toEigen(const yarp::sig::Vector & vec)
{
    return Eigen::Map<const Eigen::VectorXd>(vec.data(),vec.size());
}

/***************************************************************************/
Filter::Filter(const Vector &num, const Vector &den, const Vector &y0)
{
    b=toEigen(num);
    a=toEigen(den);

    m=b.size(); uold.resize(y0.size(),m-1);
    n=a.size(); yold.resize(y0.size(),m-1);

    init(y0);
}


/***************************************************************************/
void Filter::init(const Vector &y0)
{
    // otherwise use zero
        init(y0,yarp::math::zeros(y0.length()));
}


/***************************************************************************/
void Filter::init(const Vector &y0, const Vector &u0)
{
    Vector u_init(y0.length(),0.0);
    Vector y_init=y0;
    y=y0;

    double sum_b=0.0;
    for (int i=0; i<b.size(); i++)
        sum_b+=b[i];

    double sum_a=0.0;
    for (int i=0; i<a.size(); i++)
        sum_a+=a[i];

    if (fabs(sum_b)>1e-9)   // if filter DC gain is not zero
        u_init=(sum_a/sum_b)*y0;
    else
    {
        // if filter gain is zero then you need to know in advance what
        // the next input is going to be for initializing (that is u0)
        // Note that, unless y0=0, the filter output is not going to be stable
        u_init=u0;
        if (fabs(sum_a-a[0])>1e-9)
            y_init=a[0]/(a[0]-sum_a)*y;
        // if sum_a==a[0] then the filter can only be initialized to zero
    }

    for (int i=0; i<yold.cols(); i++)
        yold.col(i)=toEigen(y_init);
    yold_last_column_sample = 0;

    for (int i=0; i<uold.cols(); i++)
        uold.col(i)=toEigen(u_init);
    uold_last_column_sample = 0;
}


/***************************************************************************/
void Filter::getCoeffs(Vector &num, Vector &den)
{
    toEigen(num)=b;
    toEigen(den)=a;
}


/***************************************************************************/
void Filter::setCoeffs(const Vector &num, const Vector &den)
{
    b=toEigen(num);
    a=toEigen(den);

    uold.setZero();
    yold.setZero();

    m=b.size(); uold.resize(y.size(),m-1);
    n=a.size(); yold.resize(y.size(),n-1);

    init(y);
}


/***************************************************************************/
bool Filter::adjustCoeffs(const Vector &num, const Vector &den)
{
    if ((num.length()==(size_t)b.size()) && (den.length()==(size_t)a.size()))
    {
        (b)=toEigen(num);
        (a)=toEigen(den);

        return true;
    }
    else
        return false;
}


/***************************************************************************/
const Vector & Filter::filt(const Vector &u)
{
    toEigen(y)=b[0]*toEigen(u);

    for (size_t i=1; i<m; i++)
    {
        toEigen(y)+=b[i]*uold.col((m-i+uold_last_column_sample)%(m-1));
    }

    for (size_t i=1; i<n; i++)
    {
        toEigen(y)-=a[i]*yold.col((n-i+yold_last_column_sample)%(n-1));
    }

    toEigen(y)=(1.0/a[0])*toEigen(y);

    uold_last_column_sample++;
    uold_last_column_sample = uold_last_column_sample%uold.cols();
    uold.col(uold_last_column_sample) = toEigen(u);

    yold_last_column_sample++;
    yold_last_column_sample = yold_last_column_sample%yold.cols();
    yold.col(yold_last_column_sample) = toEigen(y);

    return y;
}


/**********************************************************************/
FirstOrderLowPassFilter::FirstOrderLowPassFilter(const double cutFrequency,
                                                 const double sampleTime,
                                                 const Vector &y0)
{
    fc=cutFrequency;
    Ts=sampleTime;
    y=y0;
    filter=NULL;
    computeCoeff();
}


/**********************************************************************/
FirstOrderLowPassFilter::~FirstOrderLowPassFilter()
{
    delete filter;
}


/***************************************************************************/
void FirstOrderLowPassFilter::init(const Vector &y0)
{
    if (filter!=NULL)
        filter->init(y0);
}


/**********************************************************************/
bool FirstOrderLowPassFilter::setCutFrequency(const double cutFrequency)
{
    if (cutFrequency<=0.0)
        return false;

    // Optimization : if the  cutoff frequency is exactly the same
    // do not update the coeffiencts
    // Note: in general the equality between two doubles is not a
    // reliable check, but in this case it make sense
    if( fc != cutFrequency )
    {
        fc=cutFrequency;
        computeCoeff();
    }

    return true;
}


/**********************************************************************/
bool FirstOrderLowPassFilter::setSampleTime(const double sampleTime)
{
    if (sampleTime<=0.0)
        return false;

    Ts=sampleTime;
    computeCoeff();

    return true;
}


/**********************************************************************/
const Vector& FirstOrderLowPassFilter::filt(const Vector &u)
{
    if (filter!=NULL)
        y=filter->filt(u);

    return y;
}


/**********************************************************************/
void FirstOrderLowPassFilter::computeCoeff()
{
    double tau=1.0/(2.0*M_PI*fc);
    Vector num=cat(Ts,Ts);
    Vector den=cat(2.0*tau+Ts,Ts-2.0*tau);

    if (filter!=NULL)
        filter->adjustCoeffs(num,den);
    else
        filter=new Filter(num,den,y);
}

