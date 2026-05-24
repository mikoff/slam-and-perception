#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/geometry/Pose3.h>


using namespace gtsam;

class LidarExtrinsicsFactor : public NoiseModelFactorN<Pose3, Pose3, Pose3> {
private:
    Pose3 measured_Z_;

public:
    LidarExtrinsicsFactor(Key key_Xi, Key key_Xj, Key key_T_BL, const Pose3& measured_Z, const SharedNoiseModel& model)
        : NoiseModelFactorN<Pose3, Pose3, Pose3>(model, key_Xi, key_Xj, key_T_BL),
          measured_Z_(measured_Z) {};

    Vector evaluateError(const Pose3& X_i, const Pose3& X_j, const Pose3& T_BL,
                         OptionalMatrixType H_Xi,
                         OptionalMatrixType H_Xj,
                         OptionalMatrixType H_T_BL) const override {
        
// 1. Compute poses and conditionally request their partials
        Matrix6 H_Pi_Xi, H_Pi_TBL;
        const Pose3 P_i = X_i.compose(T_BL, 
                                      H_Xi ? &H_Pi_Xi : nullptr, 
                                      H_T_BL ? &H_Pi_TBL : nullptr);

        Matrix6 H_Pj_Xj, H_Pj_TBL;
        const Pose3 P_j = X_j.compose(T_BL, 
                                      H_Xj ? &H_Pj_Xj : nullptr, 
                                      H_T_BL ? &H_Pj_TBL : nullptr);

        // 2. Determine if intermediate Jacobians are needed for the next steps
        const bool need_Pi = H_Xi || H_T_BL;
        const bool need_Pj = H_Xj || H_T_BL;

        Matrix6 H_pred_Pi, H_pred_Pj;
        const Pose3 predicted_Z = P_i.between(P_j, 
                                              need_Pi ? &H_pred_Pi : nullptr, 
                                              need_Pj ? &H_pred_Pj : nullptr);

        // 3. Compute error and conditionally request its partial
        Matrix6 H_err_pred;
        const Vector6 error = measured_Z_.localCoordinates(predicted_Z, 
                                                           nullptr, 
                                                           (need_Pi || need_Pj) ? &H_err_pred : nullptr);
        
        // 4. Assemble the final Jacobians using the Chain Rule
        if (H_Xi) {
            *H_Xi = H_err_pred * H_pred_Pi * H_Pi_Xi;
        }
        if (H_Xj) {
            *H_Xj = H_err_pred * H_pred_Pj * H_Pj_Xj;
        }
        if (H_T_BL) {
            *H_T_BL = H_err_pred * (H_pred_Pi * H_Pi_TBL + H_pred_Pj * H_Pj_TBL);
        }

        return error;
    }
};