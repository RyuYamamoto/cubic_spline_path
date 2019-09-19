#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_

#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>

const double SAMPLING_TIME = 0.01;

class CubicSpline
{
	public:
		CubicSpline(const double _sampling_time=0.1) : sampling_time(_sampling_time)
		{}
		double calc(double t)
		{
			int j = int(floor(t));
			if(j < 0) j = 0;
			else if(j >= ndata+1) j = ndata;

			double dt = t - j;
			double result = a[j] + (b[j] + (c[j] + d[j] * dt) * dt ) * dt;
			return result;
		}
		void init(std::vector<double> y)
		{
			this->ndata = y.size() - 1;

			for(std::size_t i=0;i<=ndata;i++)
				a.push_back(y[i]);

			for(std::size_t i=0;i<ndata;i++){
				if(i == 0 || i == ndata)
					c.push_back(0.0);
				else
					c.push_back(3.0*(a[i-1]-2.0*a[i]+a[i+1]));
			}

			w.push_back(0.0);
			for(std::size_t i=0;i<ndata;i++){
				double tmp = 4.0 - w[i-1];
				c[i] = (c[i] - c[i-1])/tmp;
				w.push_back(1.0 / tmp);
			}

			for(std::size_t i=(ndata-1);i>0;i--)
				c[i] = c[i] - c[i+1]*w[i];

			for(std::size_t i=0;i<=ndata;i++){
				if(i == ndata){
					d.push_back(0.0);
					b.push_back(0.0);
				}else{
					d.push_back((c[i+1]-c[i])/3.0);
					b.push_back(a[i+1]-a[i]-c[i]-d[i]);
				}
			}
		}
	public:
		const double sampling_time;
	private:
		std::size_t ndata;
		std::vector<double> a;
		std::vector<double> b;
		std::vector<double> c;
		std::vector<double> d;
		std::vector<double> w;
};

#endif 
