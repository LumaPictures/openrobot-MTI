
/*!
 * 
 * $Author: Cyril Roussillon (cyril.roussillon@laas.fr)
 * $Source:
 * $Revision: 0.0
 * $Date: 5 August 2010
 * 
 */

#ifndef _PERIODIC_TIMESTAMP_ESTIMATOR_HPP_
#define _PERIODIC_TIMESTAMP_ESTIMATOR_HPP_

#include <iostream>
#include <iomanip>
#include <cmath>

template<typename T> static inline T sqr(const T x) { return x*x; }


/**
This class is used for hardware that provide periodic measures but with no
timestamp associated, and for which the timestamp is measured after data reception.

At long term it estimates both the exact period (with an average on a sliding window)
and the time of the hardware (with a very simple Kalman filter), in order to
eliminate drift of the hardware clock,
At short term it uses the period to create the timestamp.

The results should not be used the first few seconds (ideally on a not loaded system)
in order to have a good idea of the minimal delay.

When setting it up for a new hardware, make sure that you tuned correctly the
Kalman filter uncertainties. If the initial provided period is very wrong, the estimator
will drift away until it has a good estimation of the period, and if measure_uncertainty
is too low it can take a while to stick back to the measures.

FIXME This file should be moved somewhere in general for openrobots
*/
class PeriodicTimestampEstimator
{
	private:
		// for estimating the period
		double sum_p, sum_p2; ///< sum periods and squared periods
		int windowSize; ///< averaging window size
		int usedSize; ///< the used size in the window
		double *window; ///< averaging window
		int pos; ///< position in the window
		double prev_t_measure; ///< the previous timestamp measure, used to compute the period
		double period; ///< the estimated period
		
		// for estimating t
		double t, T; ///< the time and its variance
		double measure_uncertainty; ///< the uncertainty of timestamp measures
		
		
	public:
		/**
		@param init_period the initial value of the period
		@param windowSize the sliding averaging window size for the period
		@param measure_uncertainty the uncertainty of timestamp measures, in number of sigmas that the period represents
		*/
		PeriodicTimestampEstimator(double init_period, int windowSize, double measure_uncertainty = 0.0002):
			windowSize(windowSize), usedSize(windowSize), pos(0), prev_t_measure(-1.), period(init_period), t(-1.), T(0.), measure_uncertainty(measure_uncertainty)
		{
			sum_p = init_period*usedSize;
			sum_p2 = sqr(init_period)*usedSize;
			window = new double[windowSize];
			for (int i = 0; i < usedSize; ++i)
				window[i] = init_period;
		}
		~PeriodicTimestampEstimator()
		{
			delete window;
		}

		/**
		@param t_measure the actal timestamp that was taken
		@param n_periods the number of periods that have elapsed since last estimation
		@param theoretical_delay the theoretical processing and communication delay (minimal) in seconds 
		between when the data is acquired by the hardware, and when it is available to the computer and can be timestamped.
		*/
		double estimate(double t_measure, int n_periods, double theoretical_delay)
		{
			t_measure -= theoretical_delay;
			if (prev_t_measure < 0.) { t = prev_t_measure = t_measure; return t; }
			
			// measure
			double period_measure = (t_measure-prev_t_measure)/n_periods;
			prev_t_measure = t_measure;

			// estimation of period
			if (pos < usedSize)
			{
				sum_p -= window[pos];
				sum_p2 -= sqr(window[pos]);
			}
			window[pos] = period_measure;
			sum_p += window[pos];
			sum_p2 += sqr(window[pos]);
			pos++;
			if (pos > usedSize) usedSize = pos;
			if (pos >= windowSize) pos = 0;
			period = sum_p / usedSize;
			
			// prediction of t
			t += period*n_periods;
			T += (sum_p2/usedSize - sqr(period)) * n_periods; // increase from variance of period
			
			// correction of t
			double this_measure_uncertainty = measure_uncertainty;
			if (usedSize < windowSize)
			{
				this_measure_uncertainty *= 10;
				if (usedSize < windowSize/2) this_measure_uncertainty *= 10;
				
			}
			double innov = t_measure - t;
			double INNOV = T + (innov > 0. ? sqr(innov / this_measure_uncertainty) : 0.0);
			double K = T / INNOV;
			
			t += K * innov;
			T *= (1.-K);
			
//std::cout << "measure " << std::setprecision(16) << t_measure << ", period " << period << ", and delay " << theoretical_delay << ", gives estimation " << std::setprecision(16) << t << std::endl;
//static int n = 0;
//std::cout << n++ << std::endl << std::flush; 
//std::cout << n++ << "\t" << std::setprecision(16) << t_measure << "\t" << std::setprecision(16) << t << "\t" << period_measure << "\t" << period << std::endl;
			
			return t;
		}

};


#endif
