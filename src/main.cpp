#include <eigen3/Eigen/Dense>
#include <LinearSys.h>

int main (){
    std::array<double, 8> a;
    LinearSys ls(a, a, a, a, 0.1);
    return 0;
}
