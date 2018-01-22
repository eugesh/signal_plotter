#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>

#include <Eigen/Eigen>
#include <unsupported/Eigen/NonLinearOptimization>

static const unsigned int N = 1000;
static const std::string filename("./data/test/samples_analytic_sin.txt");

// Implement y = y0 + A * sin(pi * (x - xc) / w)
struct LMFunctor
{
	// 'm' pairs of (x, f(x))
	Eigen::MatrixXf measuredValues;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		float y0_param = x(0);
		float A_param = x(1);
		float xc_param = x(2);
		float w_param = x(3);

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

		for (int i = 0; i < values(); i++) {
			float xValue = measuredValues(i, 0);
			float yValue = measuredValues(i, 1);

			fvec(i) = yValue - (y0_param + A_param * sin(w_param * (xValue - xc_param)));
		}
		return 0;
	}

	// Compute the jacobian of the errors
	int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fjac' has dimensions m x n
		// It will contain the jacobian of the errors, calculated numerically in this case.

		float epsilon;
		epsilon = 1e-5f;

		for (int i = 0; i < x.size(); i++) {
			Eigen::VectorXf xPlus(x);
			xPlus(i) += epsilon;
			Eigen::VectorXf xMinus(x);
			xMinus(i) -= epsilon;

			Eigen::VectorXf fvecPlus(values());
			operator()(xPlus, fvecPlus);

			Eigen::VectorXf fvecMinus(values());
			operator()(xMinus, fvecMinus);

			Eigen::VectorXf fvecDiff(values());
			fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

			fjac.block(0, i, values(), 1) = fvecDiff;
		}

		return 0;
	}

	// Number of data points, i.e. values.
	int m;

	// Returns 'm', the number of values.
	int values() const { return m; }

	// The number of parameters, i.e. inputs.
	int n;

	// Returns 'n', the number of inputs.
	int inputs() const { return n; }

};

void test_sample_creator () {
    // y = y0 + A * sin(w * x) + B * cos(w * x);

    // FILE *fp = fopen(filename.pointer, "w");

    for (unsigned int i = 0; i < N; ++i) {

    }

    // close(fp);
}

int main(int argc, char *argv[])
{
	//
	// Goal
	//
	// Given a non-linear equation: y = y0 + A * sin(pi * (x - xc) / w)
	// and 'm' data points (x1, f(x1)), (x2, f(x2)), ..., (xm, f(xm))
	// our goal is to estimate 'n' parameters (4 in this case: y0, A, xc, w)
	// using LM optimization.
	//

	//
	// Read values from file.
	// Each row has two numbers, for example: 5.50 223.70
	// The first number is the input value (5.50) i.e. the value of 'x'.
	// The second number is the observed output value (223.70),
	// i.e. the measured value of 'f(x)'.
	//

	std::ifstream infile("measured_data.txt");
	// std::ifstream infile("measured_data_no_noise.txt");

	if (!infile) {
		std::cout << "Unable to read file." << std::endl;
		return -1;
	}

	std::vector<float> x_values;
	std::vector<float> y_values;

	std::string line;
	while (getline(infile, line)){
		std::istringstream ss(line);
		float x, y;
		ss >> x >> y;
		x_values.push_back(x);
		y_values.push_back(y);
	}

	// 'm' is the number of data points.
	int m = x_values.size();

	// Move the data into an Eigen Matrix.
	// The first column has the input values, x. The second column is the f(x) values.
	Eigen::MatrixXf measuredValues(m, 2);
	for (int i = 0; i < m; i++) {
		measuredValues(i, 0) = x_values[i];
		measuredValues(i, 1) = y_values[i];
	}

	// 'n' is the number of parameters in the function.
	// y = y0 + A * sin(pi * (x - xc) / w) has 4 parameters: y0, A, xc, w
	int n = 4;

	// 'x' is vector of length 'n' containing the initial values for the parameters.
	// The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
	// The LM optimization inputs should not be confused with the x input values.
	Eigen::VectorXf x(n);
	x(0) = 0.0;             // initial value for 'y0'
	x(1) = 0.0;             // initial value for 'A'
	x(2) = 0.0;             // initial value for 'xc'
	x(3) = 0.0;             // initial value for 'w'

	//
	// Run the LM optimization
	// Create a LevenbergMarquardt object and pass it the functor.
	//

	LMFunctor functor;
	functor.measuredValues = measuredValues;
	functor.m = m;
	functor.n = n;

	Eigen::LevenbergMarquardt<LMFunctor, float> lm(functor);
	int status = lm.minimize(x);
	std::cout << "LM optimization status: " << status << std::endl;

	//
	// Results
	// The 'x' vector also contains the results of the optimization.
	//
	std::cout << "Optimization results" << std::endl;
	std::cout << "\ty0: " << x(0) << std::endl;
	std::cout << "\tA: "  << x(1) << std::endl;
	std::cout << "\txc: " << x(2) << std::endl;
	std::cout << "\tw: "  << x(3) << std::endl;

	return 0;
}
