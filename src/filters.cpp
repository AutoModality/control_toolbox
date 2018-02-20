#include <control_toolbox/filters.h>

namespace filters {

BWFilter2::BWFilter2()
{
	//
	input_[0] = 0.0;
	input_[1] = 0.0;
	input_[2] = 0.0;
	output_[0] = 0.0;
	output_[1] = 0.0;
	output_[2] = 0.0;

	cutoff_frequency_ = 12.5;
}

BWFilter2::BWFilter2(double cutoff_frequency)
{
	BWFilter2();

	cutoff_frequency_ = cutoff_frequency;
}

BWFilter2::~BWFilter2()
{
}

double BWFilter2::compute(double input, ros::Duration dt)
{
  if (dt == ros::Duration(0.0) || std::isnan(input) || std::isinf(input))
    return 0.0;

  double c;

  input_[2] = input_[1];
  input_[1] = input_[0];
  input_[0] = input;

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
	// My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
	if (cutoff_frequency_ != -1)
	{
		// Check if tan(_) is really small, could cause c = NaN
		double tan_filt = tan( (cutoff_frequency_*6.2832)*dt.toSec()/2 );

		// Avoid tan(0) ==> NaN
		if ( (tan_filt<=0.) && (tan_filt>-0.01) )
			tan_filt = -0.01;
		if ( (tan_filt>=0.) && (tan_filt<0.01) )
			tan_filt = 0.01;

		c = 1/tan_filt;
	}

	output_[2] = output_[1];
	output_[1] = output_[0];
	output_[0] = (1/(1+c*c+1.414*c))*(input_[2]+2*input_[1]+input_[0]-(c*c-1.414*c+1)*output_[2]-(-2*c*c+2)*output_[1]);
  }

  ROS_INFO_STREAM(
		  "c= " << c <<
		  ", dt= " << dt.toSec() <<
		  ", i0= " << input_[0] <<
		  ", i1= " << input_[1] <<
		  ", i2= " << input_[2] <<
		  ", o0= " << output_[0] <<
		  ", o1= " << output_[1] <<
		  ", o2= " << output_[2]
		  );

  return output_[0];
}

} // namespace
