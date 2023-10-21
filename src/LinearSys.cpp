#include <LinearSys.h>
    if (input1.size() != input2.size()) {
        throw std::invalid_argument("Size of input1 and input2 must be the same.");
    }

LinearSys::LinearSys(const std::array<double, 8> A, const std::array<double, 8> b, const std::array<double, 8> c, double dt): dt_{dt}{

    
}