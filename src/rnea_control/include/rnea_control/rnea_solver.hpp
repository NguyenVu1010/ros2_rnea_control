#ifndef RNEA_CONTROL__RNEA_SOLVER_HPP_
#define RNEA_CONTROL__RNEA_SOLVER_HPP_

#include <Eigen/Dense>
#include <vector>

namespace rnea_control {

struct LinkParams {
    double m;
    Eigen::Vector3d com;
    Eigen::Matrix3d I;
    Eigen::Vector3d p;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class rneasolver {
public:
    explicit rneasolver(const std::vector<LinkParams>& params) : params_(params) {
        n_ = params.size();
        w_.resize(n_ + 1, Eigen::Vector3d::Zero());
        dw_.resize(n_ + 1, Eigen::Vector3d::Zero());
        dv_.resize(n_ + 1, Eigen::Vector3d::Zero());
        f_ext_.resize(n_ + 1, Eigen::Vector3d::Zero());
        n_ext_.resize(n_ + 1, Eigen::Vector3d::Zero());
    }

    void solve(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, 
               const Eigen::VectorXd& ddq, Eigen::VectorXd& tau_out) {
        const Eigen::Vector3d z0(0, 0, 1);
        dv_[0] = Eigen::Vector3d(0, 0, 9.81); 

        for (int i = 0; i < n_; ++i) {
            Eigen::Matrix3d R = get_rotation_matrix(q[i]);
            Eigen::Matrix3d RT = R.transpose();
            w_[i+1] = RT * (w_[i] + z0 * dq[i]);
            dw_[i+1] = RT * (dw_[i] + z0 * ddq[i] + w_[i].cross(z0 * dq[i]));
            dv_[i+1] = RT * dv_[i] + dw_[i+1].cross(params_[i].p) + 
                       w_[i+1].cross(w_[i+1].cross(params_[i].p));
            Eigen::Vector3d dv_c = dv_[i+1] + dw_[i+1].cross(params_[i].com) + 
                                  w_[i+1].cross(w_[i+1].cross(params_[i].com));
            F_i_[i] = params_[i].m * dv_c;
            N_i_[i] = params_[i].I * dw_[i+1] + w_[i+1].cross(params_[i].I * w_[i+1]);
        }

        for (int i = n_ - 1; i >= 0; --i) {
            Eigen::Matrix3d R_next = (i < n_ - 1) ? get_rotation_matrix(q[i+1]) : Eigen::Matrix3d::Identity();
            f_ext_[i] = R_next * f_ext_[i+1] + F_i_[i];
            n_ext_[i] = R_next * n_ext_[i+1] + params_[i].p.cross(f_ext_[i]) + 
                        params_[i].com.cross(F_i_[i]) + N_i_[i];
            tau_out[i] = n_ext_[i].dot(z0);
        }
    }

private:
    int n_;
    std::vector<LinkParams> params_;
    std::vector<Eigen::Vector3d> w_, dw_, dv_, f_ext_, n_ext_;
    Eigen::Vector3d F_i_[10], N_i_[10];

    inline Eigen::Matrix3d get_rotation_matrix(double qi) {
        Eigen::Matrix3d R;
        double s = std::sin(qi), c = std::cos(qi);
        R << c, -s, 0,  s, c, 0,  0, 0, 1;
        return R;
    }
};

} // namespace rnea_control
#endif