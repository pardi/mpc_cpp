#ifndef ODE_HPP
#define ODE_HPP

enum class odeType{
    FORWARD
    // BACKWARD
    // TUSTIN
};

class ode{

    public: 
        ode(): dt_{0.0}{}
        ~ode() = default;
        void solve();

    private:
        double dt_;
        odeType odeType_;


};

#endif