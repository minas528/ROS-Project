#include "ros/ros.h"
#include <sstream>
#include "vector_transformation/Position.h"
#include "vector_transformation/Transformation.h"

class transformer {
	public:
		transformer() {
			sub = n.subscribe("transformation/input", 1000, &transformer::callback, this);
			pub = n.advertise<vector_transformation::Position>("transformation/output", 1000);

			// Test Input
			// vector_transformation::Transformation trial;
			// trial.position.x = 12;
			// trial.position.y = 31;
			// trial.angles.y = 45;
			// trial.translations.z = 121;
			// callback(trial);
		}

		void callback(const vector_transformation::Transformation request) {

			std::cout << "Processing Request" << std::endl;

			vector_transformation::Position result;
			int axis_x[] = {1, 2}, axis_y[] = {0, 2}, axis_z[] = {0, 1};
			double zero_transform[] = {0, 0};

			// Do the Rotations along the three axis
			auto matrix = makeMatrix(request.angles.x, axis_x, zero_transform);
			result = transform(matrix, request.position);

			matrix = makeMatrix(request.angles.y, axis_y, zero_transform);
			result = transform(matrix, result);

			matrix = makeMatrix(request.angles.z, axis_z, zero_transform);
			result = transform(matrix, result);

			// Do the Translation
			auto trans = request.translations;
			double translation[] = {trans.x, trans.y, trans.z};

			matrix = makeMatrix(0, axis_x, translation);
			result = transform(matrix, result);

			pub.publish(result);
			ros::spinOnce();

		}

		vector_transformation::Position transform(std::array<std::array<double, 4>, 4> matrix, vector_transformation::Position inputPos) {
			double input[] = {inputPos.x, inputPos.y, inputPos.z, 1};
			
			vector_transformation::Position result;
			result.x = matrix[0][0] * input[0] + matrix[0][1] * input[1] + matrix[0][2] * input[2] + matrix[0][3] * input[3];
			result.y = matrix[1][0] * input[0] + matrix[1][1] * input[1] + matrix[1][2] * input[2] + matrix[1][3] * input[3];
			result.z = matrix[2][0] * input[0] + matrix[2][1] * input[1] + matrix[2][2] * input[2] + matrix[2][3] * input[3];

			return result;
		}

		std::array<std::array<double, 4>, 4> makeMatrix(const double angle, const int axis[2], const double translations[3])
		{
			std::array<std::array<double, 4>, 4> result;
			for(int i = 0; i < 4; ++i)
				for(int j = 0; j < 4; ++j)
					result[i][j] = (i == j) ? 1 : 0;

			double rad = M_PI * angle / 180;
			result[axis[0]][axis[0]] = cos(rad), result[axis[1]][axis[1]] = cos(rad);
			result[axis[0]][axis[1]] = -sin(rad), result[axis[1]][axis[0]] = sin(rad);
			result[0][3] = translations[0], result[1][3] = translations[1], result[2][3] = translations[2];

			return result;
		}

	private:
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher pub;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "transformer");
	transformer tsfm;
	ros::spin();
	return 0;
}
